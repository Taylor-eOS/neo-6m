#include <Arduino.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>

HardwareSerial GPS_Serial(2);
TinyGPSPlus gps;

void setup() {
    Serial.begin(115200);
    GPS_Serial.begin(9600, SERIAL_8N1, 2, 15);
    Serial.println(F("GPS Ready"));
}

void loop() {
    while (GPS_Serial.available() > 0) {
        if (gps.encode(GPS_Serial.read())) {
            if (gps.location.isValid()) {
                Serial.print(F("Lat: "));
                Serial.println(gps.location.lat(), 6);
                Serial.print(F("Lng: "));
                Serial.println(gps.location.lng(), 6);
            }
        }
    }
}
