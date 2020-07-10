#pragma once

#include <stdlib.h>
#include <string>
#include <fstream>
#include <chrono>
#include <thread>
#include <sstream>
#include <chrono>
#include <math.h>

#include <spdlog/spdlog.h>
#include "Eigen/Eigen"

#include "UTM.h"

#include "time_sync.h"

using namespace std;

struct geo_location;
struct geo_utm;

class gps_parser {
public:
    gps_parser() {}
    gps_parser(std::string path) : file_path(path), is_valid(false), terminate(false) {}
    void start_reading(time_sync& time_s);
    void terminate_process();

    //update and convert new wgs84 to utm
    void update_gps_value(geo_utm& gps);
    //update gps to new value
    void update_gps_value(geo_location& gps);

    static Eigen::Vector3d get_direction_vector(geo_utm& p1, geo_utm& p2, double altitude = 0.0);

    double lat, lon, alt;
    bool is_valid;

private:
    std::string file_path;
    long long timestamp;
    bool terminate;
};

// geo location in WGS-84 format of lat, lon, alt
struct geo_location {
public:
    //double for 15 digits of decimal precision
    double latitude;
    double longitude;
    double altitude;

    geo_location() { this->latitude = this->longitude = this->altitude = 0.0; }

    geo_location(double lat, double lon, double alt)
        : latitude(lat), longitude(lon), altitude(alt) {}

    geo_location(std::string lat, std::string lon, std::string alt) {
        geo_location(stod(lat), stod(lon), stod(alt));
    }

    friend bool operator==(const geo_location& lhs, const geo_location& rhs) {
        return (lhs.latitude == rhs.latitude && lhs.longitude == rhs.longitude && lhs.altitude == rhs.altitude);
    }

    friend bool operator!=(const geo_location& lhs, const geo_location& rhs) {
        return (lhs.latitude != rhs.latitude || lhs.longitude != rhs.longitude || lhs.altitude != rhs.altitude);
    }

    //return wgs84 value in fmt(lat,lon,alt)
    std::string value() {
        return to_string(this->latitude) + "," + to_string(this->longitude) + "," + to_string(this->altitude);
    }

    void update_value(std::string lat, std::string lon, std::string alt) {
        this->latitude = stod(lat);
        this->longitude = stod(lon);
        this->altitude = stod(alt);
    }

    void update_value(gps_parser& gps) {
        this->latitude = gps.lat;
        this->longitude = gps.lon;
        this->altitude = gps.alt;
    }

    //geo_utm convert_gps_to_utm() {
    //    double x, y;
    //    LatLonToUTMXY(this->latitude, this->longitude, 0, x, y);
    //    return geo_utm(x, y, 0, (this->latitude >= 0) ? false : true);
    //}
};

/*
//ref: https://alephnull.net/software/gis/UTM_WGS84_C_plus_plus.shtml
*/

// UTM (universal transverse mercator) : accuracy around 50 cm (cause rounding??)
struct geo_utm {
    double x;   //Easting
    double y;   //Northing
    short zone; //for zone=0, the correct one will be computed
    bool southhemi;

    geo_utm() {
        this->x = this->y = 0.0;
        this->zone = 0;
        this->southhemi = false;
    }

    geo_utm(double x, double y, short zone = 0, bool south_hemi = false) {
        this->x = x;
        this->y = y;
        this->zone = zone;
        this->southhemi = south_hemi;
    }

    geo_utm(geo_location& gps) {
        LatLonToUTMXY(gps.latitude, gps.longitude, 0, this->x, this->y);
        (gps.latitude >= 0) ? this->southhemi = false : this->southhemi = true;
    }

    friend bool operator==(const geo_utm& lhs, const geo_utm& rhs) {
        return (lhs.x == rhs.x && lhs.y == rhs.y && lhs.southhemi == rhs.southhemi);
    }

    friend bool operator!=(const geo_utm& lhs, const geo_utm& rhs) {
        return (lhs.x != rhs.x || lhs.y != rhs.y || lhs.southhemi != rhs.southhemi);
    }

    geo_location convert_utm_to_gps(double alt = 0.0) {
        double lat, lon;
        UTMXYToLatLon(this->x, this->y, this->zone, this->southhemi, lat, lon);
        return geo_location(lat, lon, alt);
    }

    void update_value(gps_parser& gps) {
        this->x = gps.lat;
        this->y = gps.lon;
        this->zone = gps.alt;
    }

    //return UTM value in string fmt(x:northing,y:easting,south_hemi)
    inline string value() {
        return to_string(this->x) + "," + to_string(this->y) + "," + to_string(this->southhemi);
    }

    // update this utm from source utm
    void copy_from(geo_utm* src) {
        this->x = src->x;
        this->y = src->y;
        this->southhemi = src->southhemi;
    }

    //returns euclidean distance (m)
    inline double distance_from(geo_utm* target) {
        return std::sqrt(std::pow(this->x - target->x, 2) + std::pow(this->y - target->y, 2));
    }

    //returns squared euclidean distance (m)
    inline double sq_distance_from(geo_utm* target) {
        return std::pow(this->x - target->x, 2) + std::pow(this->y - target->y, 2);
    }
};