#include <Arduino.h>
#include "DisplayTerminal.h"
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
DisplayTerminal terminal(5, 17, 16);

String padded(int value) {
    String str;
    if (value < 10) str = F("0");
    str += String(value);
    return str;
}

void printGPSData() {
    String latStr = String(F("Lat: ")) + String(gps.location.isValid() ? gps.location.lat() : 0.0, 6);
    terminal.print(latStr);
    String lonStr = String(F("Lon: ")) + String(gps.location.isValid() ? gps.location.lng() : 0.0, 6);
    terminal.print(lonStr);
    String ageStr = String(F("Last fix age: ")) + String(gps.location.isValid() ? gps.location.age() : 0) + String(F("ms"));
    terminal.print(ageStr);
    String altStr = String(F("Alt: ")) + String(gps.altitude.isValid() ? gps.altitude.meters() : 0.0) + String(F("m"));
    terminal.print(altStr);
    String speedStr = String(F("Speed: ")) + String(gps.speed.isValid() ? gps.speed.kmph() : 0.0) + String(F(" km/h"));
    terminal.print(speedStr);
    String courseStr = String(F("Course: ")) + String(gps.course.isValid() ? gps.course.deg() : 0.0) + String(F(" deg"));
    terminal.print(courseStr);
    String timeStr;
    if (gps.time.isValid()) {
        timeStr = padded(gps.time.hour()) + String(F(":")) + padded(gps.time.minute()) + String(F(":")) + padded(gps.time.second());
    } else {
        timeStr = F("00:00:00");
    }
    terminal.print(timeStr);
    String dateStr;
    if (gps.date.isValid()) {
        dateStr = String(F("Date: ")) + String(gps.date.year()) + String(F("/")) + padded(gps.date.month()) + String(F("/")) + padded(gps.date.day());
    } else {
        dateStr = F("Date: 0000/00/00");
    }
    terminal.print(dateStr);
    String satsStr = String(F("Sats: ")) + String(gps.satellites.isValid() ? gps.satellites.value() : 0);
    terminal.print(satsStr);
    String fixTypeStr;
    if (!gps.location.isValid()) {
        fixTypeStr = F("No fix");
    } else if (gps.altitude.isValid()) {
        fixTypeStr = F("3D fix");
    } else {
        fixTypeStr = F("2D fix");
    }
    terminal.print(String(F("Fix type: ")) + fixTypeStr);
    String hdopStr = String(F("HDOP: ")) + String(gps.hdop.isValid() ? gps.hdop.hdop() : 0.0, 2);
    terminal.print(hdopStr);
    terminal.print(String(F("-")));
}

void setup() {
    Serial.begin(SERIAL_BAUD);
    terminal.init();
    terminal.print(F("Ready"));
    GPS_Serial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
}

void loop() {
    while (GPS_Serial.available() > 0) {
        gps.encode(GPS_Serial.read());
    }
    if (lastPrint == 0 || millis() - lastPrint >= PRINT_INTERVAL) {
        printGPSData();
        lastPrint = millis();
    }
}

