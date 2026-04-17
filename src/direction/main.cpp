#include <Arduino.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>
#include "target_position.h"
#include "navigation.h"

const int TFT_CS   = 5;
const int TFT_DC   = 17;
const int TFT_RST  = 16;
const int BTN_PIN  = 25;
const unsigned long UPDATE_INTERVAL  = 1000UL;
const unsigned long SERIAL_BAUD      = 115200;
const unsigned long GPS_BAUD         = 9600;
const unsigned long DEBOUNCE_MS      = 50UL;
const int TEXT_SCALE = 2;
const int CHAR_W     = 6 * TEXT_SCALE;
const int CHAR_H     = 8 * TEXT_SCALE;
const int MAX_COLS   = 160;
const int BTN_SKIP_PIN = 26;
bool lastSkipState = HIGH;
Adafruit_ST7735 tft(TFT_CS, TFT_DC, TFT_RST);
HardwareSerial GPS_Serial(2);
TinyGPSPlus gps;

unsigned long lastPrint       = 0;
unsigned long lastBtnTime     = 0;
bool firstRun                 = true;
bool waypointsInitialized     = false;
bool lastBtnState             = HIGH;
int currentRoute              = 0;

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

void activateRoute(double lat, double lon) {
    const RouteEntry& r = routes[currentRoute];
    nav_init();
    nav_waypoints_init(r.points, r.count, lat, lon);
    waypointsInitialized = true;
}

void checkButton() {
    bool state = digitalRead(BTN_PIN);
    if (lastBtnState == HIGH && state == LOW) {
        unsigned long now = millis();
        if (now - lastBtnTime >= DEBOUNCE_MS) {
            lastBtnTime = now;
            currentRoute = (currentRoute + 1) % routeCount;
            waypointsInitialized = false;
        }
    }
    lastBtnState = state;

    bool skipState = digitalRead(BTN_SKIP_PIN);
    if (lastSkipState == HIGH && skipState == LOW) {
        nav_waypoints_skip();
    }
    lastSkipState = skipState;
}

void drawNavigation(int &row) {
    char buf[MAX_COLS + 1];
    if (!gps.location.isValid()) {
        tftLine(row++, "No GPS");
        return;
    }
    double lat = gps.location.lat();
    double lon = gps.location.lng();
    if (!waypointsInitialized) {
        activateRoute(lat, lon);
    }
    nav_waypoints_update(lat, lon);
    snprintf(buf, sizeof(buf), "Route: %s", routes[currentRoute].name);
    tftLine(row++, buf);
    if (nav_waypoints_done()) {
        tftLineColored(row++, "** ARRIVED **", ST7735_GREEN);
        return;
    }
    if (nav_waypoints_just_advanced()) {
        tftLineColored(row++, "** WAYPOINT **", ST7735_GREEN);
    } else {
        double dist = nav_waypoints_distance(lat, lon);
        snprintf(buf, sizeof(buf), "WP %d/%d  %dm",
            nav_waypoints_index() + 1,
            nav_waypoints_count(),
            (int)dist);
        tftLine(row++, buf);
    }
    GeoPoint target = nav_waypoints_current();
    nav_add_point(lat, lon);
    if (nav_has_heading()) {
        double correction = nav_get_correction(lat, lon, target.lat, target.lon);
        snprintf(buf, sizeof(buf), "Turn: %.0f", correction);
        tftLine(row++, buf);
    } else {
        tftLine(row++, "Move to init");
    }
}

void drawGPSData() {
    int row = 0;
    char buf[MAX_COLS + 1];
    double lat = gps.location.isValid() ? gps.location.lat() : 0.0;
    double lon = gps.location.isValid() ? gps.location.lng() : 0.0;
    snprintf(buf, sizeof(buf), "%.6f", lat);
    tftLine(row++, buf);
    snprintf(buf, sizeof(buf), "%.6f", lon);
    tftLine(row++, buf);
    drawNavigation(row);
}

void setup() {
    Serial.begin(SERIAL_BAUD);
    pinMode(BTN_SKIP_PIN, INPUT_PULLUP);
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
    while (GPS_Serial.available() > 0) {
        gps.encode(GPS_Serial.read());
    }
    checkButton();
    if (firstRun || millis() - lastPrint >= UPDATE_INTERVAL) {
        firstRun = false;
        drawGPSData();
        lastPrint = millis();
    }
}
