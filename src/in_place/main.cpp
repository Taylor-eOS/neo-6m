#include <Arduino.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>

const int TFT_CS  = 5;
const int TFT_DC  = 17;
const int TFT_RST = 16;
//const int GPS_RX_PIN = 4;
//const int GPS_TX_PIN = 33;
const unsigned long PRINT_INTERVAL = 3000UL;
const unsigned long SERIAL_BAUD = 115200;
const unsigned long GPS_BAUD = 9600;
const unsigned long LOOP_DELAY = 200;
const int CHAR_W = 6;
const int CHAR_H = 8;
const int MAX_COLS = 26;
Adafruit_ST7735 tft(TFT_CS, TFT_DC, TFT_RST);
HardwareSerial GPS_Serial(2);
TinyGPSPlus gps;
unsigned long lastPrint = 0;
bool firstRun = true;

void tftLine(int row, const char* buf) {
    tft.setCursor(0, row * CHAR_H);
    tft.setTextColor(ST7735_WHITE, ST7735_BLACK);
    int len = strlen(buf);
    tft.print(buf);
    for (int i = len; i < MAX_COLS; i++) {
        tft.print(' ');
    }
}

void drawPosition(int &row) {
    char buf[MAX_COLS + 1];
    double lat = gps.location.isValid() ? gps.location.lat() : 0.0;
    double lon = gps.location.isValid() ? gps.location.lng() : 0.0;
    snprintf(buf, sizeof(buf), "Lat:  %.6f", lat);
    tftLine(row++, buf);
    snprintf(buf, sizeof(buf), "Lon:  %.6f", lon);
    tftLine(row++, buf);
    double absLat = fabs(lat);
    int latDeg = (int)absLat;
    double latMinFull = (absLat - latDeg) * 60.0;
    int latMin = (int)latMinFull;
    double latSec = (latMinFull - latMin) * 60.0;
    char latDir = lat >= 0 ? 'N' : 'S';
    snprintf(buf, sizeof(buf), "  %d*%d'%.2f\"%c", latDeg, latMin, latSec, latDir);
    tftLine(row++, buf);
    double absLon = fabs(lon);
    int lonDeg = (int)absLon;
    double lonMinFull = (absLon - lonDeg) * 60.0;
    int lonMin = (int)lonMinFull;
    double lonSec = (lonMinFull - lonMin) * 60.0;
    char lonDir = lon >= 0 ? 'E' : 'W';
    snprintf(buf, sizeof(buf), "  %d*%d'%.2f\"%c", lonDeg, lonMin, lonSec, lonDir);
    tftLine(row++, buf);
    snprintf(buf, sizeof(buf), "Age:  %lums", gps.location.isValid() ? gps.location.age() : 0UL);
    tftLine(row++, buf);
}

void drawMotion(int &row) {
    char buf[MAX_COLS + 1];
    snprintf(buf, sizeof(buf), "Alt:  %.1f m", gps.altitude.isValid() ? gps.altitude.meters() : 0.0);
    tftLine(row++, buf);
    snprintf(buf, sizeof(buf), "Spd:  %.1f km/h", gps.speed.isValid() ? gps.speed.kmph() : 0.0);
    tftLine(row++, buf);
    snprintf(buf, sizeof(buf), "Crs:  %.1f deg", gps.course.isValid() ? gps.course.deg() : 0.0);
    tftLine(row++, buf);
}

void drawDateTime(int &row) {
    char buf[MAX_COLS + 1];
    if (gps.time.isValid()) {
        snprintf(buf, sizeof(buf), "Time: %02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());
    } else {
        snprintf(buf, sizeof(buf), "Time: --:--:--");
    }
    tftLine(row++, buf);
    if (gps.date.isValid()) {
        snprintf(buf, sizeof(buf), "Date: %04d/%02d/%02d", gps.date.year(), gps.date.month(), gps.date.day());
    } else {
        snprintf(buf, sizeof(buf), "Date: ----/--/--");
    }
    tftLine(row++, buf);
}

void drawSignal(int &row) {
    char buf[MAX_COLS + 1];
    snprintf(buf, sizeof(buf), "Sats: %u", gps.satellites.isValid() ? gps.satellites.value() : 0u);
    tftLine(row++, buf);
    const char* fixType = !gps.location.isValid() ? "No fix" : gps.altitude.isValid() ? "3D fix" : "2D fix";
    snprintf(buf, sizeof(buf), "Fix:  %s", fixType);
    tftLine(row++, buf);
    snprintf(buf, sizeof(buf), "HDOP: %.2f", gps.hdop.isValid() ? gps.hdop.hdop() : 0.0);
    tftLine(row++, buf);
}

void drawGPSData() {
    int row = 0;
    drawPosition(row);
    drawMotion(row);
    drawDateTime(row);
    drawSignal(row);
}

void setup() {
    Serial.begin(SERIAL_BAUD);
    tft.initR(INITR_BLACKTAB);
    tft.setRotation(1);
    tft.fillScreen(ST7735_BLACK);
    tft.setTextSize(1);
    tft.setTextWrap(false);
    tft.setTextColor(ST7735_WHITE, ST7735_BLACK);
    tft.setCursor(0, 0);
    tft.print("Ready");
    GPS_Serial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
}

void loop() {
    while (GPS_Serial.available() > 0) {
        gps.encode(GPS_Serial.read());
    }
    if (firstRun || millis() - lastPrint >= PRINT_INTERVAL) {
        firstRun = false;
        drawGPSData();
        lastPrint = millis();
    }
    delay(LOOP_DELAY);
}
