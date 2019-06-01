#include "TorchyImu.h"

#import <photonic.h>

using namespace photonic;

/**
  Hardware wrappers
*/
Imu *imu;
Barometer *barometer;
TelemetryHeap *heap;

/**
  Data storage
*/
const int HISTORY_SIZE = 10;
history<float> vertical_accel_history(HISTORY_SIZE);

void setup() {
  // Initialize hardware wrappers
  imu = new TorchyImu();
  barometer = new BMP085Barometer();
  heap = new TelemetryHeap();

  imu->initialize();
  barometer->initialize();

  // Configure Photonic
  photonic_configure(ROCKET_IGNITION_G_TRIGGER, 3.0);
  photonic_configure(ROCKET_NO_IGNITION_GRACE_PERIOD, 60.0 * 10);
  photonic_configure(ROCKET_MICROCONTROLLER, TEENSY_31);
  photonic_configure(ROCKET_AUTOMATIC_BURNOUT, 5.0);
  photonic_configure(ROCKET_PRIMARY_IMU, imu);
  photonic_configure(ROCKET_PRIMARY_BAROMETER, barometer);
  photonic_configure(ROCKET_TELEMETRY_HEAP, heap);
  photonic_configure(ROCKET_VERTICAL_ACCEL_HISTORY, &vertical_accel_history);

  // Block until liftoff
  wait_for_liftoff();
}

void loop() {
  read_sensors();
}

void read_sensors() {
  imu->update();
  barometer->update();

  vertical_accel_history.add(imu->get_acc_z());
}
