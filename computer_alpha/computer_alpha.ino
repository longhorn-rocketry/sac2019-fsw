// STL redefinitions that Arduino can't build without
namespace std {
  void __throw_bad_alloc() {
  #ifdef GROUND_TEST
    Serial.println("Failed to allocate memory!");
  #endif
  }

  void __throw_length_error(char const *e) {
  #ifdef GROUND_TEST
    Serial.println("Bad vector length!");
  #endif
  }
}

#define TELEMETRY_SERIAL Serial1 // Serial line that Beta listens on
#define LLC_OE 2 // Logic Level Converter enable pin OUTPUT
#define BNO055_INT 16 // IMU interrupt pin INPUT
#define BNO055_ADDR 17 // IMU address pin, set low to 0x28, high for 0x29 OUTPUT
#define P1_L 21 // Ejection charge 1 OUTPUT
#define P2_L 22 // Ejection charge 2 OUTPUT
#define S1_l 3 // Servo 1 PWM OUTPUT
#define S1_2 4 // OUTPUT
#define S1_3 5 // OUTPUT
#define S1_4 6 // OUTPUT
#define S1_5 20 // OUTPUT
#define STAT1 13 // Status LED 1 OUTPUT
#define STAT2 14 // Status LED 2 OUTPUT
#define BZZR 15 // Buzzer OUTPUT
#define AIRBRAKE_SERVO_MIN -1 // TODO
#define AIRBRAKE_SERVO_MAX -1 // TODO
#define GROUND_TEST // TODO: REMOVE BEFORE FLIGHT

#define TELEMETRY_TOKEN_READINGS (byte)0
#define TELEMETRY_TOKEN_TIMESTAMPS (byte)1

#include "torchy_imu.h"

#include <aimbot.h>
#include <memory>
#include <photonic.h>
#include <Servo.h>

using namespace photonic;

// Event timing
Metronome mtr_airbrake_control(10); // 10 Hz
bool event_burnout = false;
bool event_apogee = false;

// Hardware interfaces
Imu *accelerometer;
Barometer *barometer;
AirbrakeController *aimbot;
Servo servo1, servo2, servo3, servo4, servo5;

// Data storage
const int HISTORY_SIZE = 20;
history<float> vertical_accel_history(HISTORY_SIZE);

// Rocket parameters
float rocket_drag_coeff = 0.46;
float rocket_radius = 0.0762;
float rocket_airbrake_area = 0.0036;
float rocket_dry_mass = 34.874;

// Rocket state
Metronome mtr_state_update(25); // 25 Hz
KalmanFilter state_estimator;
matrix rocket_state(3, 1);
float p0;
float launchpad_altitude = 1293.876; // SL altitude of Truth or Consequences, NM

// Telemetry
TelemetryHeap *heap;
Metronome mtr_record_telemetry(20); // 20 Hz
struct TelemetryLog {
  float accel;
  float accel_filtered;
  float brake_extension;
  float alt_min;
  float alt_max;
  float alt;
  float alt_filtered;
  float velocity_filtered;
  float pressure;
} telemetry_log;
const int TELEMETRY_LOGS_BEFORE_OFFLOAD = 100;
float accel_block[TELEMETRY_LOGS_BEFORE_OFFLOAD];
float accel_filtered_block[TELEMETRY_LOGS_BEFORE_OFFLOAD];
float brake_extension_block[TELEMETRY_LOGS_BEFORE_OFFLOAD];
float alt_min_block[TELEMETRY_LOGS_BEFORE_OFFLOAD];
float alt_max_block[TELEMETRY_LOGS_BEFORE_OFFLOAD];
float alt_block[TELEMETRY_LOGS_BEFORE_OFFLOAD];
float alt_filtered_block[TELEMETRY_LOGS_BEFORE_OFFLOAD];
float velocity_filtered_block[TELEMETRY_LOGS_BEFORE_OFFLOAD];
float pressure_block[TELEMETRY_LOGS_BEFORE_OFFLOAD];
float *telemetry_blocks[] = {
  accel_block,
  accel_filtered_block,
  brake_extension_block,
  alt_min_block,
  alt_max_block,
  alt_block,
  alt_filtered_block,
  velocity_filtered_block,
  pressure_block
};
const int TELEMETRY_BLOCKS = 9;
const int TELEMETRY_BUFFER_SIZE =
    TELEMETRY_LOGS_BEFORE_OFFLOAD * TELEMETRY_BLOCKS * 2; // 2 bytes per float16
int telemetry_logs_made = 0;

void setup() {
#ifdef GROUND_TEST
  Serial.println("Initializing hardware...");
#endif

  pinMode(LLC_OE, OUTPUT);
  pinMode(BNO055_INT, INPUT);
  pinMode(BNO055_ADDR, OUTPUT);
  pinMode(STAT1, OUTPUT);
  pinMode(STAT2, OUTPUT);
  pinMode(BZZR, OUTPUT);
  pinMode(P1_L, OUTPUT);
  pinMode(P2_L, OUTPUT);

  digitalWrite(LLC_OE, HIGH);
  digitalWrite(BNO055_ADDR, LOW);
  digitalWrite(P1_L, LOW);
  digitalWrite(P2_L, LOW);

  servo1.attach(S1_l);
  servo2.attach(S1_2);
  servo3.attach(S1_3);
  servo4.attach(S1_4);
  servo5.attach(S1_5);

  // Initialize hardware wrappers
  accelerometer = new TorchyImu();
  barometer = new BMP085Barometer();
  heap = new TelemetryHeap();

  bool success = accelerometer->initialize();

  if (!success) {
  #ifdef GROUND_TEST
    Serial.println("FATAL: FAILED TO CONTACT IMU");
  #endif
    chirp(3);
  }

  success = barometer->initialize();

  if (!success) {
  #ifdef GROUND_TEST
    Serial.println("FATAL: FAILED TO CONTACT BAROMETER");
  #endif
    chirp(3);
  }

  // Configure backend
  photonic_configure(ROCKET_IGNITION_G_TRIGGER, 3.0); // 3 Gs
  photonic_configure(ROCKET_NO_IGNITION_GRACE_PERIOD, 60.0 * 10); // Minimum 10 min before liftoff
  photonic_configure(ROCKET_MICROCONTROLLER, TEENSY_31);
  photonic_configure(ROCKET_AUTOMATIC_BURNOUT, 5.0); // Burnout automatically declared after 5 s
  photonic_configure(ROCKET_PRIMARY_IMU, accelerometer);
  photonic_configure(ROCKET_PRIMARY_BAROMETER, barometer);
  photonic_configure(ROCKET_TELEMETRY_HEAP, heap);
  photonic_configure(ROCKET_VERTICAL_ACCEL_HISTORY, &vertical_accel_history);
  photonic_configure(ROCKET_APOGEE_DETECTION_NEGLIGENCE, 0.15);

#ifdef GROUND_TEST
  Serial.println("Computing ground pressure...");
#endif

  // Compute ground pressure
  double sigma_p = 0;
  int n = 1000;
  for(int i = 0; i < n; i++) {
    barometer->update();
    sigma_p += barometer->get_pressure();
  }
  p0 = sigma_p / n;

#ifdef GROUND_TEST
  Serial.println("Initializing state estimator...");
#endif

  // Kalman filter setup
  state_estimator.set_delta_t(mtr_state_update.get_wavelength());
  state_estimator.set_sensor_variance(33.333, 3); // TODO: find these numbers
  state_estimator.compute_kg(1000);
  state_estimator.set_initial_estimate(0, 0, 0);

#ifdef GROUND_TEST
  Serial.println("Configuring airbrake controller...");
#endif

  // Airbrake controller setup
  AirbrakeControllerConfiguration config;
  config.target_altitude = -1; // TODO
  config.bounds_history_size = 20;
  config.enforce_bounds_history_size = true;
  config.regression_id = abc::REG_NONE;
  config.bs_profile_velocity_min = 25;
  config.bs_profile_velocity_max = 300;
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

#ifdef GROUND_TEST
  Serial.println("Setup complete. Waiting for liftoff.");
#endif
  chirp(1);

  // Block until liftoff
  wait_for_liftoff();

#ifdef GROUND_TEST
  Serial.println("Liftoff detected!");
#endif
}

void loop() {
  update_sensors();

  float t = flight_time();

  // Burnout and apogee detection
  bool block = block_control();

  // State estimation
  if (mtr_state_update.poll(t)) {
    float s = hypso(p0,
                    barometer->get_pressure(),
                    barometer->get_temperature());
    float a = accelerometer->get_acc_z();
    float mtr_dt = mtr_state_update.get_dt();
    float dt = mtr_dt == -1 ? mtr_state_update.get_wavelength() : mtr_dt;
    state_estimator.set_delta_t(dt);
    rocket_state = state_estimator.filter(s, a);
    telemetry_log.alt = s;
    telemetry_log.accel_filtered = rocket_state[2][0];
    telemetry_log.alt_filtered = rocket_state[0][0];
    telemetry_log.velocity_filtered = rocket_state[1][0];
  }

  // Airbrake control
  if (!block && mtr_airbrake_control.poll(t)) {
    float rocket_altitude = rocket_state[0][0];
    float rocket_velocity = rocket_state[1][0];

    // Set up Verlet integrator
    struct InitializationData vint_data;
    vint_data.initial_value = launchpad_altitude + rocket_altitude;
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

    telemetry_log.alt_min = alt_min;
    telemetry_log.alt_max = alt_max;

    // Compute airbrake extension and update servo
    float extension = aimbot->update(t,
                                     rocket_velocity,
                                     alt_min,
                                     alt_max);
    set_airbrake_extension(extension);
  }

  // Telemetry logging
  if (mtr_record_telemetry.poll(t) && !event_apogee) {
    // Write log values to block buffers
    int i = telemetry_logs_made;
    accel_block[i] = telemetry_log.accel;
    accel_filtered_block[i] = telemetry_log.accel_filtered;
    brake_extension_block[i] = telemetry_log.brake_extension;
    alt_min_block[i] = telemetry_log.alt_min;
    alt_max_block[i] = telemetry_log.alt_max;
    alt_block[i] = telemetry_log.alt;
    alt_filtered_block[i] = telemetry_log.alt_filtered;
    velocity_filtered_block[i] = telemetry_log.velocity_filtered;
    pressure_block[i] = telemetry_log.pressure;
    telemetry_logs_made++;

    // Offload buffer to Beta
    if (telemetry_logs_made == TELEMETRY_LOGS_BEFORE_OFFLOAD) {
      telemetry_logs_made = 0;
      byte buffer[TELEMETRY_BUFFER_SIZE + 1];
      buffer[0] = TELEMETRY_TOKEN_READINGS;
      int buffer_ind = 1;

      // Compress buffer contents to float16
      for (int i = 0; i < TELEMETRY_BLOCKS; i++) {
        float *block = telemetry_blocks[i];
        for (int j = 0; j < TELEMETRY_LOGS_BEFORE_OFFLOAD; j++) {
          float f = block[j];
          float16 f16 = Float16Compressor::compress(f);
          byte b0 = f16 & 0xFF, b1 = (f16 >> 8) & 0xFF;
          buffer[buffer_ind] = b1;
          buffer[buffer_ind + 1] = b0;
          buffer_ind += 2;
        }
      }

      // Send to Beta
      TELEMETRY_SERIAL.write(buffer, TELEMETRY_BUFFER_SIZE);
    }
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
    if (event_burnout) {}
  }

  if (!event_apogee) {
    event_apogee = check_for_apogee();
    if (event_apogee) {
      set_airbrake_extension(0);

      // Send timestamp telemetry block to Beta
      byte buffer[TELEMETRY_BUFFER_SIZE + 1];
      buffer[0] = TELEMETRY_TOKEN_TIMESTAMPS;

      // Zero the buffer
      for (int i = 0; i < TELEMETRY_BUFFER_SIZE + 1; i++)
        buffer[i] = 0;

      // Compress and enbuffer time data
      float16 t_ignition = Float16Compressor::compress(__rocket_ignition_time);
      byte b0 = t_ignition & 0xFF;
      byte b1 = (t_ignition >> 8) & 0xFF;
      buffer[1] = b1;
      buffer[2] = b0;
      float16 t_burnout = Float16Compressor::compress(__rocket_burnout_time);
      b0 = t_burnout & 0xFF;
      b1 = (t_burnout >> 8) & 0xFF;
      buffer[3] = b1;
      buffer[4] = b0;
      float16 t_apogee = Float16Compressor::compress(__rocket_apogee_time);
      b0 = t_apogee & 0xFF;
      b1 = (t_apogee >> 8) & 0xFF;
      buffer[5] = b1;
      buffer[6] = b0;

      // Send to Beta
      TELEMETRY_SERIAL.write(buffer, TELEMETRY_BUFFER_SIZE);
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

  float accel = accelerometer->get_acc_z();
  vertical_accel_history.add(accel);
  telemetry_log.accel = accel;

  telemetry_log.pressure = barometer->get_pressure();
}

/**
  @brief sets the airbrakes to (e*100)% extension
*/
void set_airbrake_extension(float e) {
  int position = (int)(AIRBRAKE_SERVO_MIN +
      (AIRBRAKE_SERVO_MAX - AIRBRAKE_SERVO_MIN) * e);
  servo1.write(position); // TODO: servo1?
  telemetry_log.brake_extension = e;
}

/**
  @brief bird time
*/
void chirp(int repeat) {
  const int PITCH = 800;
  const int ON_TIME = 100;
  const int OFF_TIME = 100;

  for (int x = 0; x < repeat; x++) {
    tone(BZZR, PITCH);
    delay(ON_TIME);
    noTone(BZZR);
    delay(OFF_TIME);
  }
}
