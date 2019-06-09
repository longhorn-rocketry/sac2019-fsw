#include "torchy_imu.h"

TorchyImu::TorchyImu() {
  bno = nullptr;
}

TorchyImu::~TorchyImu() {
  if (bno != nullptr)
    delete bno;
}

bool TorchyImu::initialize() {
  bno = new Adafruit_BNO055(55); // Not sure why 55
  return bno->begin();
}

void TorchyImu::update() {
  sensors_event_t event;
  bno->getEvent(&event, Adafruit_BNO055::VECTOR_LINEARACCEL);

  data.ax = event.acceleration.x;
  data.ay = event.acceleration.y;
  data.az = event.acceleration.z;
}
