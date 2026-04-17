#include <math.h>
#include "navigation.h"

#define MAX_POINTS 12
#define MIN_MOVE_METERS 3.0
#define MIN_TOTAL_SPAN 15.0
#define WAYPOINT_RADIUS 18.0
#define APPROACH_RADIUS 50.0

static GeoPoint buffer[MAX_POINTS];
static int bufferCount = 0;
static double lastHeading = NAN;
static const GeoPoint* wpList = nullptr;
static int wpCount = 0;
static int wpIndex = 0;
static bool justAdvanced = false;
static double correctionBias = 0.0;

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
    justAdvanced = false;
    correctionBias = 0.0;
    wpList = nullptr;
    wpCount = 0;
    wpIndex = 0;
}

void nav_add_point(double lat, double lon) {
    GeoPoint p = {lat, lon};
    if (bufferCount > 0) {
        double d = haversineDistance(buffer[bufferCount - 1], p);
        if (d < MIN_MOVE_METERS) return;
    }
    if (bufferCount >= 2) {
        GeoPoint a = buffer[bufferCount - 2];
        GeoPoint b = buffer[bufferCount - 1];
        double prevBearing = bearingBetween(a, b);
        double newBearing = bearingBetween(b, p);
        double delta = fabs(angleDiff(newBearing, prevBearing));
        bool sharpTurn = delta > 45.0;
        bool gradualTurn = false;
        if (bufferCount >= 4) {
            GeoPoint c = buffer[bufferCount - 4];
            double olderBearing = bearingBetween(c, a);
            double accumulated = fabs(angleDiff(newBearing, olderBearing));
            if (accumulated > 60.0) gradualTurn = true;
        }
        if (sharpTurn || gradualTurn) {
            buffer[0] = b;
            buffer[1] = p;
            bufferCount = 2;
            correctionBias = 0.0;
            return;
        }
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
        double t = (double)i / bufferCount;
        double w = t * t;
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
    if (hasLong) {
        if (isnan(lastHeading)) {
            lastHeading = longHeading;
        } else {
            lastHeading = blendAngles(lastHeading, longHeading, 0.2);
        }
    }
    double finalHeading = lastHeading;
    if (hasShort && !isnan(lastHeading)) {
        double w = shortStrength * 0.5;
        finalHeading = blendAngles(lastHeading, shortHeading, w);
    }
    return finalHeading;
}

double nav_get_correction(double lat, double lon, double targetLat, double targetLon) {
    double heading = nav_get_heading();
    double targetBearing = bearingBetween({lat, lon}, {targetLat, targetLon});
    double raw = angleDiff(targetBearing, heading);
    double adjusted = angleDiff(raw, correctionBias);
    if (fabs(adjusted) < 25.0) {
        correctionBias = correctionBias * 0.98 + raw * 0.02;
    }
    return adjusted;
}

DirState nav_get_state(double correction) {
    return STRAIGHT;
}

void nav_waypoints_init(const GeoPoint* points, int count, double lat, double lon) {
    wpList = points;
    wpCount = count;
    wpIndex = 0;
    justAdvanced = false;
    if (count <= 0) return;
    GeoPoint here = {lat, lon};
    int bestIndex = 0;
    double bestDist = haversineDistance(here, points[0]);
    for (int i = 1; i < count; i++) {
        double d = haversineDistance(here, points[i]);
        if (d < bestDist) {
            bestDist = d;
            bestIndex = i;
        }
    }
    wpIndex = bestIndex;
    if (wpIndex < wpCount - 1) {
        double dCurrent = haversineDistance(here, points[wpIndex]);
        double dNext = haversineDistance(here, points[wpIndex + 1]);
        if (dNext < dCurrent) {
            wpIndex++;
        }
    }
}

static void computeSegmentMetrics(GeoPoint p, GeoPoint a, GeoPoint b, double &along, double &cross) {
    double d13 = haversineDistance(a, p);
    double theta13 = toRadians(bearingBetween(a, p));
    double theta12 = toRadians(bearingBetween(a, b));
    double R = 6371000.0;
    double delta13 = d13 / R;
    double crossRad = asin(sin(delta13) * sin(theta13 - theta12));
    cross = fabs(crossRad * R);
    double alongRad = acos(cos(delta13) / cos(crossRad));
    along = alongRad * R;
    double segLen = haversineDistance(a, b);
    if (along > segLen) along = segLen;
}

void nav_waypoints_update(double lat, double lon) {
    justAdvanced = false;
    if (wpList == nullptr || wpIndex >= wpCount) return;
    GeoPoint here = {lat, lon};
    GeoPoint target = wpList[wpIndex];
    double d = haversineDistance(here, target);
    if (wpIndex > 0) {
        GeoPoint prev = wpList[wpIndex - 1];
        double along, cross;
        computeSegmentMetrics(here, prev, target, along, cross);
        double segLen = haversineDistance(prev, target);
        bool reachedByProgress = along > segLen * 0.9 && cross < 25.0;
        bool reachedByDistance = d <= WAYPOINT_RADIUS && cross < 25.0;
        if ((reachedByProgress || reachedByDistance) && wpIndex < wpCount - 1) {
            wpIndex++;
            justAdvanced = true;
            return;
        }
    }
    if (d <= WAYPOINT_RADIUS && wpIndex < wpCount - 1) {
        wpIndex++;
        justAdvanced = true;
    }
}

void nav_waypoints_skip() {
    if (wpList != nullptr && wpIndex < wpCount - 1) {
        wpIndex++;
        justAdvanced = true;
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

bool nav_waypoints_just_advanced() {
    return justAdvanced;
}

double nav_waypoints_next_bearing() {
    if (wpList == nullptr || wpIndex + 1 >= wpCount) return -1.0;
    return bearingBetween(wpList[wpIndex], wpList[wpIndex + 1]);
}

bool nav_waypoints_in_approach(double lat, double lon) {
    if (wpList == nullptr || wpIndex >= wpCount) return false;
    GeoPoint here = {lat, lon};
    return haversineDistance(here, wpList[wpIndex]) <= APPROACH_RADIUS;
}
