#pragma once
#include "navigation.h"

static const GeoPoint route_kurja[] = {
    {10.000000, 10.000000},
    {10.000000, 10.000000},
};

static const GeoPoint route_back[] = {
    {10.000000, 10.000000},
    {10.000000, 10.000000},
};

struct RouteEntry {
    const char* name;
    const GeoPoint* points;
    int count;
};

static const RouteEntry routes[] = {
    {"Kurja",  route_kurja,   sizeof(route_kurja)   / sizeof(route_kurja[0])},
    {"Airport",   route_back,    sizeof(route_back)    / sizeof(route_back[0])},
};

static const int routeCount = sizeof(routes) / sizeof(routes[0]);
