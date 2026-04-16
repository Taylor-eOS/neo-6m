#pragma once

struct GeoPoint {
    double lat;
    double lon;
};

enum DirState {
    STRAIGHT,
    LEFT,
    RIGHT,
    HARD_LEFT,
    HARD_RIGHT
};

void nav_init();
void nav_add_point(double lat, double lon);
bool nav_has_heading();
double nav_get_heading();
double nav_get_correction(double lat, double lon, double targetLat, double targetLon);
DirState nav_get_state(double correction);
