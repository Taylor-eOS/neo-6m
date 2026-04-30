#include <Arduino.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>
#include "navigation.h"

const int TFT_CS   = 5;
const int TFT_DC   = 17;
const int TFT_RST  = 16;
const unsigned long UPDATE_INTERVAL = 1000UL;
const unsigned long SERIAL_BAUD     = 115200;
const unsigned long GPS_BAUD        = 9600;
const int TEXT_SCALE = 2;
const int CHAR_H     = 8 * TEXT_SCALE;
const int MAX_COLS   = 160;

Adafruit_ST7735 tft(TFT_CS, TFT_DC, TFT_RST);
HardwareSerial GPS_Serial(2);
TinyGPSPlus gps;
unsigned long lastPrint = 0;
bool firstRun = true;

static const char* headingCardinal(double h) {
    if (h < 22.5 || h >= 337.5) return "N";
    if (h < 67.5)  return "NE";
    if (h < 112.5) return "E";
    if (h < 157.5) return "SE";
    if (h < 202.5) return "S";
    if (h < 247.5) return "SW";
    if (h < 292.5) return "W";
    return "NW";
}

void tftLine(int row, const char* buf) {
    tft.setCursor(0, row * CHAR_H);
    tft.setTextColor(ST7735_WHITE, ST7735_BLACK);
    int len = strlen(buf);
    tft.print(buf);
    for (int i = len; i < MAX_COLS; i++) tft.print(' ');
}

void tftLineColored(int row, const char* buf, uint16_t color) {
    tft.setCursor(0, row * CHAR_H);
    tft.setTextColor(color, ST7735_BLACK);
    int len = strlen(buf);
    tft.print(buf);
    for (int i = len; i < MAX_COLS; i++) tft.print(' ');
    tft.setTextColor(ST7735_WHITE, ST7735_BLACK);
}

void drawDisplay() {
    int row = 0;
    char buf[MAX_COLS + 1];
    if (!gps.location.isValid() || !gps.speed.isValid()) {
        tftLine(row++, "No GPS fix");
        tftLine(row++, "");
        tftLine(row++, "");
        tftLine(row++, "");
        tftLine(row++, "");
        return;
    }
    double lat = gps.location.lat();
    double lon = gps.location.lng();
    double rawSpeedKmh = gps.speed.kmph();
    nav_update(lat, lon, rawSpeedKmh);
    double cur  = nav_get_current_speed();
    double avg  = nav_get_avg_speed();
    double dist = nav_get_distance_km();
    unsigned long movSec = nav_get_moving_seconds();
    unsigned long hh = movSec / 3600;
    unsigned long mm = (movSec % 3600) / 60;
    unsigned long ss = movSec % 60;
    snprintf(buf, sizeof(buf), "Spd: %.1f km/h", cur);
    tftLineColored(row++, buf, ST7735_YELLOW);
    snprintf(buf, sizeof(buf), "Avg: %.1f km/h", avg);
    tftLine(row++, buf);
    snprintf(buf, sizeof(buf), "Dst: %.2f km", dist);
    tftLine(row++, buf);
    snprintf(buf, sizeof(buf), "Time: %02lu:%02lu:%02lu", hh, mm, ss);
    tftLine(row++, buf);
    if (nav_has_heading()) {
        double hdg = nav_get_heading();
        snprintf(buf, sizeof(buf), "Hdg: %.0f %s", hdg, headingCardinal(hdg));
        tftLine(row++, buf);
    } else {
        tftLine(row++, "Hdg: --");
    }
}

void setup() {
    Serial.begin(SERIAL_BAUD);
    tft.initR(INITR_BLACKTAB);
    tft.setRotation(1);
    tft.fillScreen(ST7735_BLACK);
    tft.setTextSize(TEXT_SCALE);
    tft.setTextWrap(false);
    tft.setCursor(0, 0);
    tft.print("Ready");
    GPS_Serial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    nav_init();
}

void loop() {
    while (GPS_Serial.available() > 0) gps.encode(GPS_Serial.read());
    if (firstRun || millis() - lastPrint >= UPDATE_INTERVAL) {
        firstRun = false;
        drawDisplay();
        lastPrint = millis();
    }
}
