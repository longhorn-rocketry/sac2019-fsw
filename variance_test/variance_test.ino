// STL redefinitions that Arduino can't build without
namespace std {
  void __throw_bad_alloc() {
    Serial.println("Failed to allocate memory!");
  }

  void __throw_length_error(char const *e) {
    Serial.println("Bad vector length!");
  }
}

#include "torchy_imu.h"

#include <photonic.h>

using namespace photonic;

TorchyImu bno;
BMP085Barometer bmp;
float p0;

void setup() {
  bno.initialize();
  bmp.initialize();

  double sigma_p = 0;
  int n = 1000;
  for(int i = 0; i < n; i++) {
    bmp.update();
    sigma_p += bmp.get_pressure();
  }
  p0 = sigma_p / n;

  Serial.println("Computing IMU variance...");

  n = 10000;
  history<float> accel = history<float>(n);
  for (int i = 0; i < n; i++) {
    bno.update();
    accel.add(bno.get_acc_z());
  }

  Serial.printf("var=%f\n", accel.stdev() * accel.stdev());

  Serial.println("Computing barometer variance...");

  history<float> alt = history<float>(n);
  for (int i = 0; i < n; i++) {
    bmp.update();
    float p = bmp.get_pressure();
    float t = bmp.get_temperature();
    float s = hypso(p0, p, t);
    alt.add(s);
  }

  Serial.printf("var=%f\n", alt.stdev() * alt.stdev());
}

void loop() {

}
