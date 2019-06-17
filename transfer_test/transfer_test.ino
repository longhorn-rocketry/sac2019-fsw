#define TELEMETRY_SERIAL Serial1 // Serial line that Beta listens on
#define TELEMETRY_TOKEN_READINGS (byte)0
#define TELEMETRY_TOKEN_TIMESTAMPS (byte)1
#define TELEMETRY_BUFFER_SIZE 1800

void setup() {
  byte buffer[TELEMETRY_BUFFER_SIZE + 1];
  buffer[0] = TELEMETRY_TOKEN_READINGS;

  for (int i = 1; i <= TELEMETRY_BUFFER_SIZE; i++)
    buffer[i] = 'a';

  TELEMETRY_SERIAL.write(buffer, TELEMETRY_BUFFER_SIZE + 1);

  buffer[0] = TELEMETRY_TOKEN_TIMESTAMPS;

  TELEMETRY_SERIAL.write(buffer, TELEMETRY_BUFFER_SIZE + 1);
}

void loop() {}
