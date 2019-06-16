#define S1_1 3 // Servo 1 PWM OUTPUT

#define AIRBRAKE_SERVO_MIN -1 // TODO probably ~40
#define AIRBRAKE_SERVO_MAX -1 // TODO probably ~80

#include <Servo.h>
#include <Wire.h>

Servo servo;

void setup() {
  servo.attach(S1_1);

  while (true) {
    delay(3000);
    set_airbrake_extension(0);
    delay(3000);
    set_airbrake_extension(1);
  }
}

void loop() {}

void set_airbrake_extension(float e) {
  int position = (int)(AIRBRAKE_SERVO_MIN +
      (AIRBRAKE_SERVO_MAX - AIRBRAKE_SERVO_MIN) * e);
  servo.write(position);
}
