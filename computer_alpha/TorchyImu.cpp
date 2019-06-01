#include "TorchyImu.h"

int16_t TorchyImu::read16() {
  int16_t x = 0;
  for (int i = 0; i < 2; i++)
    x = (x << 8) | Wire.read();
  return x;
}

void TorchyImu::initialize() {
  Wire.begin();

  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x10);
  Wire.endTransmission(true);

  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(AFS_SEL << 3);
  Wire.endTransmission(true);
}

void TorchyImu::update() {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 6, true);

  data.ax = ((float)read16()) / (1 << (14 - AFS_SEL));
  data.ay = ((float)read16()) / (1 << (14 - AFS_SEL));
  data.az = ((float)read16()) / (1 << (14 - AFS_SEL));
}
