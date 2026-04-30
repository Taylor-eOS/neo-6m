#pragma once

struct GeoPoint {
    double lat;
    double lon;
};

void nav_init();
void nav_update(double lat, double lon, double rawSpeedKmh);
bool nav_has_heading();
double nav_get_heading();
double nav_get_avg_speed();
double nav_get_current_speed();
double nav_get_distance_km();
unsigned long nav_get_moving_seconds();
