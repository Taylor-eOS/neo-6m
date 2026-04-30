#include <Arduino.h>
#include <math.h>
#include "navigation.h"

#define RAW_BUF_SIZE     80
#define SPEED_BUF_SIZE   600
#define SPEED_CUR_SAMPLES 5
#define MIN_MOVE_METERS  5.0
#define MIN_HEADING_SPAN 20.0
#define TURN_THRESHOLD   35.0
#define TURN_CONFIRM     4
#define MOVING_THRESHOLD 2.0

static GeoPoint rawBuf[RAW_BUF_SIZE];
static int rawCount = 0;

static float speedBuf[SPEED_BUF_SIZE];
static int speedHead = 0;
static int speedCount = 0;

static double lastHeading = NAN;
static double totalDistKm = 0.0;
static unsigned long movingSeconds = 0;
static int turnVoteCount = 0;

static double toRad(double d) { return d * M_PI / 180.0; }
static double toDeg(double r) { return r * 180.0 / M_PI; }

static double haversine(GeoPoint a, GeoPoint b) {
    double R = 6371000.0;
    double dLat = toRad(b.lat - a.lat);
    double dLon = toRad(b.lon - a.lon);
    double la1 = toRad(a.lat), la2 = toRad(b.lat);
    double h = sin(dLat/2)*sin(dLat/2) + cos(la1)*cos(la2)*sin(dLon/2)*sin(dLon/2);
    return 2.0 * R * atan2(sqrt(h), sqrt(1.0 - h));
}

static double bearing(GeoPoint a, GeoPoint b) {
    double lat1 = toRad(a.lat), lat2 = toRad(b.lat);
    double dLon = toRad(b.lon - a.lon);
    double y = sin(dLon) * cos(lat2);
    double x = cos(lat1)*sin(lat2) - sin(lat1)*cos(lat2)*cos(dLon);
    double b_ = toDeg(atan2(y, x));
    if (b_ < 0) b_ += 360.0;
    return b_;
}

static double angleDiff(double a, double b) {
    double d = a - b;
    while (d > 180) d -= 360;
    while (d < -180) d += 360;
    return d;
}

static bool computeHeadingFrom(const GeoPoint* pts, int n, double &out) {
    if (n < 3) return false;
    double span = haversine(pts[0], pts[n-1]);
    if (span < MIN_HEADING_SPAN) return false;
    double sx = 0, sy = 0, sw = 0;
    for (int i = 1; i < n; i++) {
        double d = haversine(pts[i-1], pts[i]);
        if (d < 1.0) continue;
        double b = bearing(pts[i-1], pts[i]);
        double t = (double)i / n;
        double w = t * t * d;
        sx += cos(toRad(b)) * w;
        sy += sin(toRad(b)) * w;
        sw += w;
    }
    if (sw == 0) return false;
    double h = toDeg(atan2(sy, sx));
    if (h < 0) h += 360.0;
    out = h;
    return true;
}

void nav_init() {
    rawCount = 0;
    speedHead = 0;
    speedCount = 0;
    lastHeading = NAN;
    totalDistKm = 0.0;
    movingSeconds = 0;
    turnVoteCount = 0;
}

void nav_update(double lat, double lon, double rawSpeedKmh) {
    if (rawSpeedKmh >= MOVING_THRESHOLD) {
        movingSeconds++;
        totalDistKm += rawSpeedKmh / 3600.0;
        speedBuf[speedHead] = (float)rawSpeedKmh;
        speedHead = (speedHead + 1) % SPEED_BUF_SIZE;
        if (speedCount < SPEED_BUF_SIZE) speedCount++;
    }

    GeoPoint p = { lat, lon };
    if (rawCount > 0) {
        double d = haversine(rawBuf[rawCount - 1], p);
        if (d < MIN_MOVE_METERS) return;
    }

    if (rawCount < RAW_BUF_SIZE) {
        rawBuf[rawCount++] = p;
    } else {
        for (int i = 1; i < RAW_BUF_SIZE; i++) rawBuf[i-1] = rawBuf[i];
        rawBuf[RAW_BUF_SIZE - 1] = p;
    }

    if (rawCount >= 8) {
        double shortH, longH;
        bool hasShort = computeHeadingFrom(rawBuf + rawCount - 4, 4, shortH);
        bool hasLong  = computeHeadingFrom(rawBuf, rawCount, longH);
        if (hasShort && hasLong) {
            if (fabs(angleDiff(shortH, longH)) > TURN_THRESHOLD) {
                turnVoteCount++;
            } else {
                if (turnVoteCount > 0) turnVoteCount--;
            }
            if (turnVoteCount >= TURN_CONFIRM) {
                int keep = 5;
                int start = rawCount - keep;
                for (int i = 0; i < keep; i++) rawBuf[i] = rawBuf[start + i];
                rawCount = keep;
                turnVoteCount = 0;
            }
        }
    }

    double newH;
    if (computeHeadingFrom(rawBuf, rawCount, newH)) {
        if (isnan(lastHeading)) {
            lastHeading = newH;
        } else {
            double x = cos(toRad(lastHeading)) * 0.85 + cos(toRad(newH)) * 0.15;
            double y = sin(toRad(lastHeading)) * 0.85 + sin(toRad(newH)) * 0.15;
            double h = toDeg(atan2(y, x));
            if (h < 0) h += 360.0;
            lastHeading = h;
        }
    }
}

bool nav_has_heading() {
    return !isnan(lastHeading);
}

double nav_get_heading() {
    return lastHeading;
}

double nav_get_avg_speed() {
    if (speedCount == 0) return 0.0;
    double sum = 0;
    for (int i = 0; i < speedCount; i++) sum += speedBuf[i];
    return sum / speedCount;
}

double nav_get_current_speed() {
    if (speedCount == 0) return 0.0;
    int n = speedCount < SPEED_CUR_SAMPLES ? speedCount : SPEED_CUR_SAMPLES;
    double sum = 0;
    int base = (speedHead - n + SPEED_BUF_SIZE) % SPEED_BUF_SIZE;
    for (int i = 0; i < n; i++)
        sum += speedBuf[(base + i) % SPEED_BUF_SIZE];
    return sum / n;
}

double nav_get_distance_km() {
    return totalDistKm;
}

unsigned long nav_get_moving_seconds() {
    return movingSeconds;
}
