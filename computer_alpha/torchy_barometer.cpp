#include "torchy_barometer.h"

void TorchyBarometer::initialize() {
  baro.begin();

  // Compute ground pressure
  ground_pressure = 0;
  int n = 1000;
  for (int i = 0; i < n; i++) {
    update();
    ground_pressure += data.pressure;
  }
  ground_pressure /= n;
  update();
}

void TorchyBarometer::update() {
  // Second highest level of sampling precision for the BMP180, only causes
  // ~14 ms delay in reading according to API
  const char PRESSURE_OVERSAMPLING = 2;

  double p, t;
  char status = baro.startTemperature();
  bool ok = false;
  if (status != 0) {
    delay(status);
    status = baro.getTemperature(t);
    if (status != 0) {
      status = baro.startPressure(PRESSURE_OVERSAMPLING);
      if (status != 0) {
        delay(status);
        status = baro.getPressure(p, t);
        ok = status != 0;
      }
    }
  }

  if (ok) {
    data.pressure = p;
    data.temperature = t;
    data.altitude = baro.altitude(p, ground_pressure);
  }
}

float TorchyBarometer::get_ground_pressure() {
  return ground_pressure;
}
