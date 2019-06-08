#ifndef TORCHY_BAROMETER_H
#define TORCHY_BAROMETER_H

#include <photonic.h>
#include <SFE_BMP180.h>

using namespace photonic;

class TorchyBarometer : public Barometer {
protected:
	SFE_BMP180 baro;
  float ground_pressure, ground_altitude;

public:
	void initialize();

	void update();

	float get_ground_pressure();
};

#endif
