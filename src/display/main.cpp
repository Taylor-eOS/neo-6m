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
    if (gps.location.isValid()) {
        String latStr = String(F("Lat: ")) + String(gps.location.lat(), 6);
        terminal.print(latStr);
        String lonStr = String(F("Lon: ")) + String(gps.location.lng(), 6);
        terminal.print(lonStr);
        String ageStr = String(F("Last fix age: ")) + String(gps.location.age()) + String(F("ms"));
        terminal.print(ageStr);
    }
    if (gps.altitude.isValid()) {
        String altStr = String(F("Alt: ")) + String(gps.altitude.meters()) + String(F("m"));
        terminal.print(altStr);
    }
    if (gps.speed.isValid()) {
        String speedStr = String(F("Speed: ")) + String(gps.speed.kmph()) + String(F(" km/h"));
        terminal.print(speedStr);
    }
    if (gps.course.isValid()) {
        String courseStr = String(F("Course: ")) + String(gps.course.deg()) + String(F(" deg"));
        terminal.print(courseStr);
    }
    if (gps.time.isValid()) {
        String timeStr = padded(gps.time.hour()) + String(F(":")) + padded(gps.time.minute()) + String(F(":")) + padded(gps.time.second());
        terminal.print(timeStr);
    }
    if (gps.date.isValid()) {
        String dateStr = String(F("Date: ")) + String(gps.date.year()) + String(F("/")) + padded(gps.date.month()) + String(F("/")) + padded(gps.date.day());
        terminal.print(dateStr);
    }
    if (gps.satellites.isValid()) {
        String satsStr = String(F("Sats: ")) + String(gps.satellites.value());
        terminal.print(satsStr);
    }
    if (gps.hdop.isValid()) {
        String hdopStr = String(F("HDOP: ")) + String(gps.hdop.hdop(), 2);
        terminal.print(hdopStr);
    }
    terminal.print(String(F("-")));
}

void setup() {
    Serial.begin(SERIAL_BAUD);
    terminal.init();
    terminal.print(F("GPS Ready"));
    GPS_Serial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
}

void loop() {
    while (GPS_Serial.available() > 0) {
        if (gps.encode(GPS_Serial.read())) {
            if (gps.location.isValid() && (lastPrint == 0 || millis() - lastPrint >= PRINT_INTERVAL)) {
                printGPSData();
                lastPrint = millis();
            }
        }
    }
}
