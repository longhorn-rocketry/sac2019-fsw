#include <photonic.h>

using namespace photonic;

BMP085Barometer b;
float p0;

void setup() {
  bool success = b.initialize();
  if (!success)
    Serial.println("FAILED TO CONTACT BAROMETER");

  double sigma_p = 0;
  int n = 1000;
  for(int i = 0; i < n; i++) {
    b.update();
    sigma_p += b.get_pressure();
  }
  p0 = sigma_p / n;
}

void loop() {
  b.update();
  float p = b.get_pressure();
  float t = b.get_temperature();
  // float s = b.get_altitude();
  float s_computed = hypso(p0, p, t);
  float rho = igl_density(p * 100, t + 273.15);
  Serial.printf("GL altitude: %f\n", s_computed); // Should be around 0
  Serial.printf("Air density: %f\n", rho); // Should be around 0.07
}
