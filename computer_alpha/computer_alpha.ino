// STL redefinitions that Arduino can't build without
namespace std {
  void __throw_bad_alloc() {
    Serial.println("Failed to allocate memory!");
  }

  void __throw_length_error(char const *e) {
    Serial.println("Bad vector length!");
  }
}

#include "torchy_barometer.h"
#include "torchy_imu.h"

#include <aimbot.h>
#include <memory>
#include <photonic.h>

using namespace photonic;

// Event timing
Metronome mtr_airbrake_control(10); // 10 Hz
bool event_burnout = false;
bool event_apogee = false;

// Hardware interfaces
TorchyImu *accelerometer;
TorchyBarometer *barometer;
TelemetryHeap *heap;
AirbrakeController *aimbot;

// Data storage
const int HISTORY_SIZE = 10;
history<float> vertical_accel_history(HISTORY_SIZE);

// Rocket parameters
float rocket_drag_coeff = -1; // TODO
float rocket_radius = 0.0762;
float rocket_airbrake_area = -1; // TODO
float rocket_dry_mass = -1; // TODO

// Rocket state
Metronome mtr_state_update(25); // 25 Hz
KalmanFilter state_estimator;
matrix rocket_state(3, 1);
float s0;

void setup() {
  Serial.println("Initializing hardware...");

  // Initialize hardware wrappers
  accelerometer = new TorchyImu();
  barometer = new TorchyBarometer();
  heap = new TelemetryHeap();

  accelerometer->initialize();
  barometer->initialize();

  // Configure backend
  photonic_configure(ROCKET_IGNITION_G_TRIGGER, 3.0); // 3 Gs
  photonic_configure(ROCKET_NO_IGNITION_GRACE_PERIOD, 60.0 * 10); // Minimum 10 min before liftoff
  photonic_configure(ROCKET_MICROCONTROLLER, TEENSY_31);
  photonic_configure(ROCKET_AUTOMATIC_BURNOUT, 5.0); // Burnout automatically declared after 5 s
  photonic_configure(ROCKET_PRIMARY_IMU, accelerometer);
  photonic_configure(ROCKET_PRIMARY_BAROMETER, barometer);
  photonic_configure(ROCKET_TELEMETRY_HEAP, heap);
  photonic_configure(ROCKET_VERTICAL_ACCEL_HISTORY, &vertical_accel_history);

  Serial.println("Initializing state estimator...");

  // Kalman filter setup
  state_estimator.set_delta_t(mtr_state_update.get_wavelength());
  state_estimator.set_sensor_variance(33.333, 3); // TODO: find these numbers
  state_estimator.compute_kg(1000);
  state_estimator.set_initial_estimate(0, 0, 0);

  Serial.println("Configuring airbrake controller...");

  // Airbrake controller setup
  AirbrakeControllerConfiguration config;
  config.target_altitude = -1; // TODO
  config.bounds_history_size = 20;
  config.enforce_bounds_history_size = true;
  config.regression_id = abc::REG_NONE;
  config.bs_profile_velocity_min = -1; // TODO
  config.bs_profile_velocity_max = -1; // TODO
  config.bs_profile_step_min = 0.075;
  config.bs_profile_step_max = 0.1;
  config.bs_profile_exp = -1;
  config.bsc_history_size = 10;
  config.bsc_thresh_osc = 0.5;
  config.bsc_thresh_stb = 0.1;
  config.bsc_down_profile_velocity_min = config.bs_profile_velocity_min;
  config.bsc_down_profile_velocity_max = config.bs_profile_velocity_max;
  config.bsc_down_profile_weight_min = 0.5;
  config.bsc_down_profile_weight_max = 0.7;
  config.bsc_down_profile_exp = -1;
  config.bsc_up_profile_velocity_min = config.bs_profile_velocity_min;
  config.bsc_up_profile_velocity_max = config.bs_profile_velocity_max;
  config.bsc_up_profile_weight_min = 1.3;
  config.bsc_up_profile_weight_max = 1.5;
  config.bsc_up_profile_exp = -1;
  aimbot = new AirbrakeController(config);

  Serial.println("Setup complete. Waiting for liftoff.");

  // Block until liftoff
  wait_for_liftoff();
}

void loop() {
  update_sensors();

  float t = flight_time();

  // Burnout and apogee detection
  bool block = block_control();

  // State estimation
  if (mtr_state_update.poll(t)) {
    float s = hypso(barometer->get_ground_pressure(),
                    barometer->get_pressure(),
                    barometer->get_temperature());
    float a = accelerometer->get_acc_z();
    rocket_state = state_estimator.filter(s, a);
  }

  // Airbrake control
  if (!block && mtr_airbrake_control.poll(t)) {
    float rocket_altitude = rocket_state[0][0];
    float rocket_velocity = rocket_state[1][0];

    // Set up Verlet integrator
    struct InitializationData vint_data;
    vint_data.initial_value = rocket_altitude;
    vint_data.start_time = t;
    vint_data.initial_velocity = rocket_velocity;
    VerletIntegrator vint = VerletIntegrator(vint_data);

    struct AccelerationCalculationData acalc_data;
    acalc_data.drag_coefficient = rocket_drag_coeff;
    acalc_data.radius = rocket_radius;
    acalc_data.base_mass = rocket_dry_mass;

    // Compute minimum altitude curve
    float rad_big = sqrt(rocket_radius * rocket_radius +
                         rocket_airbrake_area / M_PI);
    acalc_data.radius = rad_big;
    float alt_min = (float)vint.SimulateApogeeEuler(0.01, acalc_data);

    // Compute maximum altitude curve
    acalc_data.radius = rocket_radius;
    float alt_max = (float)vint.SimulateApogeeEuler(0.01, acalc_data);

    // Compute airbrake extension and update servo
    float extension = aimbot->update(t,
                                     rocket_velocity,
                                     alt_min,
                                     alt_max);
    set_airbrake_extension(extension);
  }
}

/**
  Gets whether or not to block airbrake control. Also checks for burnout and
  apogee events, and runs one-time procedures that only happen on those events.

  @return if airbrake control should be blocked
*/
bool block_control() {
  if (!event_burnout) {
    event_burnout = check_for_burnout();
    if (event_burnout) {
      // TODO: something, probably nothing
    }
  }

  if (!event_apogee) {
    event_apogee = check_for_apogee();
    if (event_apogee) {
      // TODO: send all telemetry to beta, retract airbrakes
    }
  }

  return !event_burnout || event_apogee;
}

/**
  Reads all sensors and updates histories and vehicle state.
*/
void update_sensors() {
  accelerometer->update();
  barometer->update();

  vertical_accel_history.add(accelerometer->get_acc_z());
}

/**
  @brief sets the airbrakes to (e*100)% extension
*/
void set_airbrake_extension(float e) {
  // TODO
}
