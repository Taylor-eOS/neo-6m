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
    HARD_RIGHT,
    ARRIVING
};

void nav_init();
void nav_add_point(double lat, double lon);
bool nav_has_heading();
double nav_get_heading();
double nav_get_correction(double lat, double lon, double targetLat, double targetLon);
DirState nav_get_state(double correction);
void nav_waypoints_init(const GeoPoint* points, int count, double lat, double lon);
void nav_waypoints_update(double lat, double lon);
void nav_waypoints_skip();
bool nav_waypoints_done();
int nav_waypoints_index();
int nav_waypoints_count();
GeoPoint nav_waypoints_current();
double nav_waypoints_distance(double lat, double lon);
bool nav_waypoints_just_advanced();
double nav_waypoints_next_bearing();
bool nav_waypoints_in_approach(double lat, double lon);
