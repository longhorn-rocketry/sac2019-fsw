#include <Adafruit_GPS.h>

#define fc_serial Serial3
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

Adafruit_GPS gps(&fc_serial);
String gps_data[2];

void setup() {
  Serial.println("Initializing hardware...");

  pinMode(XBEE_RES, OUTPUT);
  pinMode(XBEE_STAT, INPUT);
  pinMode(XBEE_DTR, OUTPUT);
  pinMode(XBEE_RTS, OUTPUT);
  pinMode(XBEE_CTS, INPUT);
  pinMode(XBEE_RSSI, INPUT);

  pinMode(GPS_PPS, INPUT);
  pinMode(GPS_FIX, INPUT);
  pinMode(GPS_EN, OUTPUT);

  pinMode(S1_E, OUTPUT);
  pinMode(S2_E, OUTPUT);
  pinMode(S3_E, OUTPUT);
  pinMode(S4_E, OUTPUT);
  pinMode(S5_E, OUTPUT);

  digitalWrite(XBEE_RES, LOW);
  digitalWrite(S1_E, HIGH); // Set the servo power pins high whenever most of the other stuff is done initializing
  digitalWrite(S2_E, HIGH);
  digitalWrite(S3_E, HIGH);
  digitalWrite(S4_E, HIGH);
  digitalWrite(S5_E, HIGH);

  Serial.println("Connecting to GPS...");

  gps.begin(9600);
  gps.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);
  gps.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
}

void loop() {
  // put your main code here, to run repeatedly:

}

void read_gps() {
  char c;
  while (!gps.newNMEAreceived()) {
    c = gps.read();
  }

  gps.parse(gps.lastNMEA());
  String NMEA1 = gps.lastNMEA();

  while (!gps.newNMEAreceived()) {
    c = gps.read();
  }
  gps.parse(gps.lastNMEA());
  String NMEA2 = gps.lastNMEA();

  gps_data[0] = NMEA1;
  gps_data[1] = NMEA2;
}
