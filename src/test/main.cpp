#include <Arduino.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>

const int GPS_RX_PIN = 4;
const int GPS_TX_PIN = 15;
const unsigned long PRINT_INTERVAL = 3000UL;
const unsigned long SERIAL_BAUD = 115200;
const unsigned long GPS_BAUD = 9600;
HardwareSerial GPS_Serial(2);
TinyGPSPlus gps;
unsigned long lastPrint = 0;

void printPadded(int value) {
    if (value < 10) Serial.print(F("0"));
    Serial.print(value);
}

void printGPSData() {
    if (gps.location.isValid()) {
        Serial.print(F("Lat: "));
        Serial.println(gps.location.lat(), 6);
        Serial.print(F("Lon: "));
        Serial.println(gps.location.lng(), 6);
        Serial.print(F("Last fix age: "));
        Serial.print(gps.location.age());
        Serial.println(F("ms"));
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
        printPadded(gps.time.hour());
        Serial.print(F(":"));
        printPadded(gps.time.minute());
        Serial.print(F(":"));
        printPadded(gps.time.second());
        Serial.println();
    }
    if (gps.date.isValid()) {
        Serial.print(F("Date: "));
        Serial.print(gps.date.year());
        Serial.print(F("/"));
        printPadded(gps.date.month());
        Serial.print(F("/"));
        printPadded(gps.date.day());
        Serial.println();
    }
    if (gps.satellites.isValid()) {
        Serial.print(F("Sats: "));
        Serial.println(gps.satellites.value());
    }
    if (gps.hdop.isValid()) {
        Serial.print(F("HDOP: "));
        Serial.println(gps.hdop.hdop(), 2);
    }
}

void setup() {
    Serial.begin(SERIAL_BAUD);
    GPS_Serial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    Serial.println(F("GPS Ready"));
}

void loop() {
    while (GPS_Serial.available() > 0) {
        if (gps.encode(GPS_Serial.read())) {
            if (gps.location.isValid() && (lastPrint == 0 || millis() - lastPrint >= PRINT_INTERVAL)) {
                printGPSData();
                Serial.println(F("-"));
                lastPrint = millis();
            }
        }
    }
}
