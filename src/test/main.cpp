#include <Arduino.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>

HardwareSerial GPS_Serial(2);
TinyGPSPlus gps;

void setup() {
    Serial.begin(115200);
    GPS_Serial.begin(9600, SERIAL_8N1, 4, 15);
    Serial.println(F("GPS Ready"));
}

void loop() {
    while (GPS_Serial.available() > 0) {
        if (gps.encode(GPS_Serial.read())) {
            if (gps.location.isValid()) {
                Serial.print(F("Lat: "));
                Serial.println(gps.location.lat(), 6);
                Serial.print(F("Lon: "));
                Serial.println(gps.location.lng(), 6);
            }
            if (gps.altitude.isValid()) {
                Serial.print(F("Alt: "));
                Serial.print(gps.altitude.meters());
                Serial.println(F("m"));
            }
            if (gps.speed.isValid()) {
                Serial.print(F("Speed: "));
                Serial.print(gps.speed.kmph());
                Serial.println(F(" km/h"));
            }
            if (gps.course.isValid()) {
                Serial.print(F("Course: "));
                Serial.print(gps.course.deg());
                Serial.println(F(" deg"));
            }
            if (gps.time.isValid()) {
                if (gps.time.hour() < 10) Serial.print(F("0"));
                Serial.print(gps.time.hour());
                Serial.print(F(":"));
                if (gps.time.minute() < 10) Serial.print(F("0"));
                Serial.print(gps.time.minute());
                Serial.print(F(":"));
                if (gps.time.second() < 10) Serial.print(F("0"));
                Serial.println(gps.time.second());
            }
            if (gps.date.isValid()) {
                Serial.print(F("Date: "));
                Serial.print(gps.date.year());
                Serial.print(F("/"));
                if (gps.date.month() < 10) Serial.print(F("0"));
                Serial.print(gps.date.month());
                Serial.print(F("/"));
                if (gps.date.day() < 10) Serial.print(F("0"));
                Serial.println(gps.date.day());
            }
            if (gps.satellites.isValid()) {
                Serial.print(F("Sats: "));
                Serial.println(gps.satellites.value());
            }
            if (gps.hdop.isValid()) {
                Serial.print(F("HDOP: "));
                Serial.println(gps.hdop.hdop());
            }
            Serial.print(F("Last fix age: "));
            Serial.print(gps.location.age());
            Serial.println(F("ms"));
        }
    }
}
