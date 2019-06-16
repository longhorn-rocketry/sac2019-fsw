#include <Adafruit_GPS.h>
#include <SD.h>
#include <SPI.h>

#define GPS_SERIAL Serial3 // Communication with GPS
#define TELEMETRY_SERIAL Serial2 // Incoming telemetry from Alpha
#define XBEE_RES 2 // Set HIGH to reset XBEE OUTPUT
#define XBEE_STAT 17 // XBee status pin INPUT
#define XBEE_DTR 23 // XBee sleep control OUTPUT
#define XBEE_RTS 22 // XBee request to send flow control OUTPUT
#define XBEE_CTS 39 // XBee clear to send flow control INPUT
#define XBEE_RSSI 16 // XBee connection strength INPUT

#define GPS_PPS 5 // GPS pulse per second (PPS) pin INPUT
#define GPS_FIX 6 // GPS signal fix found pin INPUT
#define GPS_EN 29 // GPS enable pin OUTPUT

#define S1_E 24 // Servo 1 power enable (hopefully this works)
#define S2_E 25 // OUTPUT
#define S3_E 26 // OUTPUT
#define S4_E 27 // OUTPUT
#define S5_E 28 // OUTPUT

#define CHIP_SELECT BUILTIN_SDCARD
#define ALPHA_TELEMETRY_BUFFER_SIZE 1801 // Size of buffer received from Alpha

#define TELEMETRY_TOKEN_READINGS (byte)0
#define TELEMETRY_TOKEN_TIMESTAMPS (byte)1

#define GROUND_TEST // TODO remove before flight

Adafruit_GPS gps(&GPS_SERIAL);

int telemetry_block_index = 0;

void setup() {
#ifdef GROUND_TEST
  Serial.begin(9600);
	while (!Serial);
#endif

  GPS_SERIAL.begin(9600);
  while (!GPS_SERIAL);

  TELEMETRY_SERIAL.begin(9600);
  while (!TELEMETRY_SERIAL);

  SD.begin(CHIP_SELECT);

#ifdef GROUND_TEST
  Serial.println("Initializing hardware...");
#endif

  // Configure GPS pins
  pinMode(XBEE_RES, OUTPUT);
  pinMode(XBEE_STAT, INPUT);
  pinMode(XBEE_DTR, OUTPUT);
  pinMode(XBEE_RTS, OUTPUT);
  pinMode(XBEE_CTS, INPUT);
  pinMode(XBEE_RSSI, INPUT);
  digitalWrite(XBEE_RES, LOW);

  pinMode(GPS_PPS, INPUT);
  pinMode(GPS_FIX, INPUT);
  pinMode(GPS_EN, OUTPUT);

  // Configure servo pins
  pinMode(S1_E, OUTPUT);
  pinMode(S2_E, OUTPUT);
  pinMode(S3_E, OUTPUT);
  pinMode(S4_E, OUTPUT);
  pinMode(S5_E, OUTPUT);

  // Raise servo lines high
  digitalWrite(S1_E, HIGH);
  digitalWrite(S2_E, HIGH);
  digitalWrite(S3_E, HIGH);
  digitalWrite(S4_E, HIGH);
  digitalWrite(S5_E, HIGH);

#ifdef GROUND_TEST
  Serial.println("Connecting to GPS...");
#endif

  gps.begin(9600);
  gps.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);
  gps.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
}

void loop() {
  // Attempt a GPS read
  read_gps();

  // Store telemetry blocks received from Alpha
  if (TELEMETRY_SERIAL.peek() != -1) {
    byte token = TELEMETRY_SERIAL.read();

    // Read telemetry block into local buffer
    byte buffer[ALPHA_TELEMETRY_BUFFER_SIZE - 1];
    for (int i = 0; i < ALPHA_TELEMETRY_BUFFER_SIZE - 1; i++)
      buffer[i] = TELEMETRY_SERIAL.read();

    // Dump buffer into a new block file
    char *filename = nullptr;

    // File name is based on token type for convenience
    if (token == TELEMETRY_TOKEN_READINGS) {
      filename = new char[7];
      filename[0] = 'b';
      filename[1] = 'l';
      filename[2] = 'o';
      filename[3] = 'c';
      filename[4] = 'k';
      filename[5] = 48 + telemetry_block_index;
      filename[6] = 0;
    } else if (token == TELEMETRY_TOKEN_TIMESTAMPS) {
      filename = new char[6];
      filename[0] = 't';
      filename[1] = 'i';
      filename[2] = 'm';
      filename[3] = 'e';
      filename[4] = 's';
      filename[5] = 0;
    }

    File file = SD.open(filename, FILE_WRITE);
    file.write(buffer, ALPHA_TELEMETRY_BUFFER_SIZE - 1);

    // Tidy up
    file.close();
    telemetry_block_index++;
    if (filename != nullptr)
      delete filename;
  }
}

/**
  @brief read NMEA sentences from GPS and write them to SD
*/
void read_gps() {
  if (gps.newNMEAreceived()) {
    char c = gps.read();

    gps.parse(gps.lastNMEA());
    String nmea = gps.lastNMEA() + "\n";

    File file = SD.open("nmea", FILE_WRITE);
    file.write(nmea.c_str(), nmea.length());
    file.close();
  }
}
