#include <math.h>
#include "navigation.h"

#define MAX_POINTS 12
#define MIN_MOVE_METERS 3.0
#define MIN_TOTAL_SPAN 8.0
#define DEAD_ZONE 20.0
#define HARD_TURN 70.0
#define HYST 10.0
#define WAYPOINT_RADIUS 20.0

static GeoPoint buffer[MAX_POINTS];
static int bufferCount = 0;
static double lastHeading = NAN;
static DirState currentState = STRAIGHT;

static const GeoPoint* wpList = nullptr;
static int wpCount = 0;
static int wpIndex = 0;

double toRadians(double deg) {
    return deg * M_PI / 180.0;
}

double toDegrees(double rad) {
    return rad * 180.0 / M_PI;
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

void nav_init() {
    bufferCount = 0;
    lastHeading = NAN;
    currentState = STRAIGHT;
}

void nav_add_point(double lat, double lon) {
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

static bool computeHeading(double &headingOut) {
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

static bool computeShortHeading(double &headingOut, double &strengthOut) {
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

static double blendAngles(double a, double b, double w) {
    double x = cos(toRadians(a)) * (1.0 - w) + cos(toRadians(b)) * w;
    double y = sin(toRadians(a)) * (1.0 - w) + sin(toRadians(b)) * w;
    double result = atan2(y, x);
    result = toDegrees(result);
    if (result < 0) result += 360.0;
    return result;
}

bool nav_has_heading() {
    double longHeading;
    if (computeHeading(longHeading)) {
        lastHeading = longHeading;
        return true;
    }
    return !isnan(lastHeading);
}

double nav_get_heading() {
    double longHeading;
    double shortHeading;
    double shortStrength;
    bool hasLong = computeHeading(longHeading);
    bool hasShort = computeShortHeading(shortHeading, shortStrength);
    if (hasLong) lastHeading = longHeading;
    double finalHeading = lastHeading;
    if (hasShort && !isnan(lastHeading)) {
        finalHeading = blendAngles(lastHeading, shortHeading, shortStrength);
    }
    return finalHeading;
}

double nav_get_correction(double lat, double lon, double targetLat, double targetLon) {
    double heading = nav_get_heading();
    double targetBearing = bearingBetween({lat, lon}, {targetLat, targetLon});
    return angleDiff(targetBearing, heading);
}

DirState nav_get_state(double correction) {
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

void nav_waypoints_init(const GeoPoint* points, int count, double lat, double lon) {
    wpList = points;
    wpCount = count;
    wpIndex = 0;
    if (count <= 0) return;
    GeoPoint here = {lat, lon};
    int best = 0;
    double bestDist = haversineDistance(here, points[0]);
    for (int i = 1; i < count; i++) {
        double d = haversineDistance(here, points[i]);
        if (d < bestDist) {
            bestDist = d;
            best = i;
        }
    }
    wpIndex = best;
}

void nav_waypoints_update(double lat, double lon) {
    if (wpList == nullptr || wpIndex >= wpCount) return;
    GeoPoint here = {lat, lon};
    double d = haversineDistance(here, wpList[wpIndex]);
    if (d <= WAYPOINT_RADIUS && wpIndex < wpCount - 1) {
        wpIndex++;
    }
}

void nav_waypoints_skip() {
    if (wpList != nullptr && wpIndex < wpCount - 1) {
        wpIndex++;
    }
}

bool nav_waypoints_done() {
    return wpList == nullptr || wpIndex >= wpCount;
}

int nav_waypoints_index() {
    return wpIndex;
}

int nav_waypoints_count() {
    return wpCount;
}

GeoPoint nav_waypoints_current() {
    if (wpList == nullptr || wpIndex >= wpCount) return {0.0, 0.0};
    return wpList[wpIndex];
}

double nav_waypoints_distance(double lat, double lon) {
    if (wpList == nullptr || wpIndex >= wpCount) return -1.0;
    GeoPoint here = {lat, lon};
    return haversineDistance(here, wpList[wpIndex]);
}
