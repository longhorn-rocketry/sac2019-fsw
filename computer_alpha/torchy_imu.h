#ifndef TORCHY_IMU_H
#define TORCHY_IMU_H

#define AFS_SEL 0

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <photonic.h>
#include <Wire.h>

using namespace photonic;

class TorchyImu : public Imu {
protected:
  Adafruit_BNO055 *bno;

public:
  TorchyImu();

  ~TorchyImu();

	void initialize();

	void update();
};

#endif
