#include <Arduino.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>
#include "target_position.h"

const int TFT_CS  = 5;
const int TFT_DC  = 17;
const int TFT_RST = 16;
const unsigned long UPDATE_INTERVAL = 1000UL;
const unsigned long SERIAL_BAUD = 115200;
const unsigned long GPS_BAUD = 9600;
const int TEXT_SCALE = 2;
const int CHAR_W = 6 * TEXT_SCALE;
const int CHAR_H = 8 * TEXT_SCALE;
const int MAX_COLS = 160;
#define MAX_POINTS 12
#define MIN_MOVE_METERS 3.0
#define MIN_TOTAL_SPAN 8.0
#define DEAD_ZONE 20.0
#define HARD_TURN 70.0
#define HYST 10.0
Adafruit_ST7735 tft(TFT_CS, TFT_DC, TFT_RST);
HardwareSerial GPS_Serial(2);
TinyGPSPlus gps;
unsigned long lastPrint = 0;
bool firstRun = true;

struct GeoPoint {
    double lat;
    double lon;
};

GeoPoint buffer[MAX_POINTS];
int bufferCount = 0;
double lastHeading = NAN;

enum DirState {
    STRAIGHT,
    LEFT,
    RIGHT,
    HARD_LEFT,
    HARD_RIGHT
};

DirState currentState = STRAIGHT;

double toRadians(double deg) {
    return deg * PI / 180.0;
}

double toDegrees(double rad) {
    return rad * 180.0 / PI;
}

double haversineDistance(GeoPoint a, GeoPoint b) {
    double R = 6371000.0;
    double dLat = toRadians(b.lat - a.lat);
    double dLon = toRadians(b.lon - a.lon);
    double lat1 = toRadians(a.lat);
    double lat2 = toRadians(b.lat);
    double h = sin(dLat/2)*sin(dLat/2) + cos(lat1)*cos(lat2)*sin(dLon/2)*sin(dLon/2);
    return 2 * R * atan2(sqrt(h), sqrt(1-h));
}

double bearingBetween(GeoPoint a, GeoPoint b) {
    double lat1 = toRadians(a.lat);
    double lat2 = toRadians(b.lat);
    double dLon = toRadians(b.lon - a.lon);
    double y = sin(dLon) * cos(lat2);
    double x = cos(lat1)*sin(lat2) - sin(lat1)*cos(lat2)*cos(dLon);
    double brng = atan2(y, x);
    brng = toDegrees(brng);
    if (brng < 0) brng += 360.0;
    return brng;
}

double angleDiff(double a, double b) {
    double d = a - b;
    while (d > 180) d -= 360;
    while (d < -180) d += 360;
    return d;
}

void addPoint(double lat, double lon) {
    GeoPoint p = {lat, lon};
    if (bufferCount > 0) {
        double d = haversineDistance(buffer[bufferCount - 1], p);
        if (d < MIN_MOVE_METERS) return;
    }
    if (bufferCount < MAX_POINTS) {
        buffer[bufferCount++] = p;
    } else {
        for (int i = 1; i < MAX_POINTS; i++) buffer[i-1] = buffer[i];
        buffer[MAX_POINTS - 1] = p;
    }
}

bool computeHeading(double &headingOut) {
    if (bufferCount < 3) return false;
    double totalSpan = haversineDistance(buffer[0], buffer[bufferCount - 1]);
    if (totalSpan < MIN_TOTAL_SPAN) return false;
    double sumX = 0;
    double sumY = 0;
    double weightSum = 0;
    for (int i = 1; i < bufferCount; i++) {
        double d = haversineDistance(buffer[i-1], buffer[i]);
        if (d < 1.0) continue;
        double b = bearingBetween(buffer[i-1], buffer[i]);
        double w = (double)i / bufferCount;
        sumX += cos(toRadians(b)) * w;
        sumY += sin(toRadians(b)) * w;
        weightSum += w;
    }
    if (weightSum == 0) return false;
    double avg = atan2(sumY, sumX);
    avg = toDegrees(avg);
    if (avg < 0) avg += 360.0;
    headingOut = avg;
    return true;
}

bool computeShortHeading(double &headingOut, double &strengthOut) {
    if (bufferCount < 2) return false;
    GeoPoint a = buffer[bufferCount - 2];
    GeoPoint b = buffer[bufferCount - 1];
    double d = haversineDistance(a, b);
    if (d < MIN_MOVE_METERS) return false;
    headingOut = bearingBetween(a, b);
    double strength = d / 6.0;
    if (strength > 1.0) strength = 1.0;
    strengthOut = strength;
    return true;
}

double blendAngles(double a, double b, double w) {
    double x = cos(toRadians(a)) * (1.0 - w) + cos(toRadians(b)) * w;
    double y = sin(toRadians(a)) * (1.0 - w) + sin(toRadians(b)) * w;
    double result = atan2(y, x);
    result = toDegrees(result);
    if (result < 0) result += 360.0;
    return result;
}

DirState updateState(double correction) {
    DirState next = currentState;
    switch (currentState) {
        case STRAIGHT:
            if (correction > HARD_TURN) next = HARD_RIGHT;
            else if (correction > DEAD_ZONE) next = RIGHT;
            else if (correction < -HARD_TURN) next = HARD_LEFT;
            else if (correction < -DEAD_ZONE) next = LEFT;
            break;
        case RIGHT:
            if (correction < DEAD_ZONE - HYST) next = STRAIGHT;
            else if (correction > HARD_TURN + HYST) next = HARD_RIGHT;
            break;
        case LEFT:
            if (correction > -DEAD_ZONE + HYST) next = STRAIGHT;
            else if (correction < -HARD_TURN - HYST) next = HARD_LEFT;
            break;
        case HARD_RIGHT:
            if (correction < HARD_TURN - HYST) next = RIGHT;
            break;
        case HARD_LEFT:
            if (correction > -HARD_TURN + HYST) next = LEFT;
            break;
    }
    currentState = next;
    return next;
}

void tftLine(int row, const char* buf) {
    tft.setCursor(0, row * CHAR_H);
    tft.setTextColor(ST7735_WHITE, ST7735_BLACK);
    int len = strlen(buf);
    tft.print(buf);
    for (int i = len; i < MAX_COLS; i++) tft.print(' ');
}

void drawNavigation(int &row) {
    char buf[MAX_COLS + 1];
    if (!gps.location.isValid()) {
        tftLine(row++, "No GPS");
        return;
    }
    double lat = gps.location.lat();
    double lon = gps.location.lng();
    addPoint(lat, lon);
    double longHeading;
    bool hasLong = computeHeading(longHeading);
    double shortHeading;
    double shortStrength;
    bool hasShort = computeShortHeading(shortHeading, shortStrength);
    if (hasLong) lastHeading = longHeading;
    double finalHeading = lastHeading;
    if (hasShort && !isnan(lastHeading)) {
        finalHeading = blendAngles(lastHeading, shortHeading, shortStrength);
    }
    double targetBearing = bearingBetween({lat, lon}, {targetLat, targetLon});
    if (!isnan(finalHeading)) {
        double correction = angleDiff(targetBearing, finalHeading);
        DirState state = updateState(correction);
        snprintf(buf, sizeof(buf), "Turn: %.0f", correction);
        tftLine(row++, buf);
        const char* text = "STRAIGHT";
        if (state == LEFT) text = "LEFT";
        if (state == RIGHT) text = "RIGHT";
        if (state == HARD_LEFT) text = "HARD LEFT";
        if (state == HARD_RIGHT) text = "HARD RIGHT";
        tftLine(row++, text);
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
    tft.initR(INITR_BLACKTAB);
    tft.setRotation(1);
    tft.fillScreen(ST7735_BLACK);
    tft.setTextSize(TEXT_SCALE);
    tft.setTextWrap(false);
    tft.setCursor(0, 0);
    tft.print("Ready");
    GPS_Serial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
}

void loop() {
    while (GPS_Serial.available() > 0) {
        gps.encode(GPS_Serial.read());
    }
    if (firstRun || millis() - lastPrint >= UPDATE_INTERVAL) {
        firstRun = false;
        drawGPSData();
        lastPrint = millis();
    }
}
