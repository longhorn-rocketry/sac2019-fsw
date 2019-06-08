#include "torchy_barometer.h"
#include "torchy_imu.h"

#include <memory>
#include <photonic.h>

using namespace photonic;

// Event timing
Metronome mtr_airbrake_control(10); // 10 Hz
bool event_burnout = false;
bool event_apogee = false;

// Hardware wrappers
Imu *accelerometer;
Barometer *barometer;
TelemetryHeap *heap;

// Data storage
const int HISTORY_SIZE = 10;
history<float> vertical_accel_history(HISTORY_SIZE);

// Rocket state
Metronome mtr_state_update(25); // 25 Hz
KalmanFilter state_estimator;
matrix rocket_state(3, 1);

void setup() {
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

  // Kalman filter setup
  state_estimator.set_delta_t(mtr_state_update.get_wavelength());
  state_estimator.set_sensor_variance(33.333, 3); // TODO: find these numbers
  state_estimator.compute_kg(1000);
  state_estimator.set_initial_estimate(0, 0, 0);

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
    float s = barometer->get_altitude();
    float a = accelerometer->get_acc_z();
    rocket_state = state_estimator.filter(s, a);
  }

  // Airbrake control
  if (!block && mtr_airbrake_control.poll(t)) {
    // TODO: do airbrake control
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
