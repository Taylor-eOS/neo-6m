#include <Arduino.h>
#include <HardwareSerial.h>

HardwareSerial GPS_Serial(2);

void setup() {
  Serial.begin(115200);
  GPS_Serial.begin(9600, SERIAL_8N1, 2, 15);
}

void loop() {
  while (GPS_Serial.available()) {
    Serial.write(GPS_Serial.read());
  }
}
