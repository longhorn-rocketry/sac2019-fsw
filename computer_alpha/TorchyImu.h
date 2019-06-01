#ifndef TORCHY_IMU_H
#define TORCHY_IMU_H

#define AFS_SEL 0

#include "Wire.h"

#include <photonic.h>

using namespace photonic;

class TorchyImu : public Imu {
protected:
  int16_t read16();

public:
	void initialize();

	void update();
};

#endif
