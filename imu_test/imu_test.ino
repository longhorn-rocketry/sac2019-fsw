#include "torchy_imu.h"

#include <photonic.h>

using namespace photonic;

Imu *bno;

void setup() {
  bno = new TorchyImu();
  bool success = bno->initialize();
  if (!success)
    Serial.println("FAILED TO CONTACT IMU");
}

void loop() {
  bno->update();
  float acc_z = bno->get_acc_z();
  Serial.printf("Z accel: %f\n", acc_z);
}
