#include "gps_fusion.h"

using namespace Eigen;


gps_fusion::gps_fusion() {
	this->initialized = false;
	this->timestampLast = getCurrentTimestamp();
}

bool gps_fusion::isInitialized() {
	return this->initialized;
}

void gps_fusion::init() {

	slamTranslationPrevious = Eigen::Vector3f::Zero();
	slamOrientation = Eigen::Quaternionf::Identity();

	this->initialized = true;
}

unsigned long gps_fusion::getCurrentTimestamp() {
	return (unsigned long)std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

unsigned long gps_fusion::getLastTimestamp() {
	return this->timestampLast;
}

void gps_fusion::setLastTimestampToCurrent() {
	this->timestampLast = getCurrentTimestamp();
}

double gps_fusion::getDeltaTimeInSeconds() {
	return fabs((double)(this->getCurrentTimestamp() - this->getLastTimestamp())) / 1000.0;
}

/*
	 CONVERT to and from Geodetic (lat, lon, height) to ECEF coordinates (x, y, z)
	 Ported to C++ from: http://danceswithcode.net/engineeringnotes/geodetic_to_ecef/geodetic_to_ecef.html
	 
*/

//Convert Earth-Centered-Earth-Fixed (ECEF) to lat, Lon, Altitude
//Input is a three element array containing x, y, z in meters
//Returned array contains lat and lon in radians, and altitude in meters
//void gps_fusion::ecef_to_geo(double& ecef_x, double& ecef_y, double& ecef_z, double& geo_lat, double& geo_lon, double& geo_alt) {
//	double zp, w2, w, r2, r, s2, c2, u, v, s, ss, c, g, rg, rf, f, m, p;
//
//	zp = fabs(ecef_z);
//	w2 = ecef_x * ecef_x + ecef_y * ecef_y;
//	w = sqrt(w2);
//	r2 = w2 + ecef_z * ecef_z;
//	r = sqrt(r2);
//	geo_lon = atan2(ecef_y, ecef_x); //Lon (final)
//	s2 = ecef_z * ecef_z / r2;
//	c2 = w2 / r2;
//	u = a2 / r;
//	v = a3 - a4 / r;
//	if (c2 > 0.3) {
//		s = (zp / r) * (1.0 + c2 * (a1 + u + s2 * v) / r);
//		geo_lat = asin(s); //Lat
//		ss = s * s;
//		c = sqrt(1.0 - ss);
//	}
//	else {
//		c = (w / r) * (1.0 - s2 * (a5 - u - c2 * v) / r);
//		geo_lat = acos(c); //Lat
//		ss = 1.0 - c * c;
//		s = sqrt(ss);
//	}
//	g = 1.0 - e2 * ss;
//	rg = a / sqrt(g);
//	rf = a6 * rg;
//	u = w - rg * c;
//	v = zp - rf * s;
//	f = c * u + s * v;
//	m = c * v - s * u;
//	p = m / (rf / g + f);
//	geo_lat = geo_lat + p;     //Lat
//	geo_alt = f + m * p / 2.0; //Altitude
//	if (ecef_z < 0.0) {
//		geo_lat *= -1.0; //Lat
//	}
//}
//
////Convert Lat, Lon, Altitude to Earth-Centered-Earth-Fixed (ECEF)
////Input is a three element array containing lat, lon (rads) and alt (m)
////Returned array contains x, y, z in meters
//void gps_fusion::geo_to_ecef(double& geo_lat, double& geo_lon, double& geo_alt, double& ecef_x, double& ecef_y, double& ecef_z) {
//	double n = a / sqrt(1 - e2 * sin(geo_lat) * sin(geo_lat));
//	ecef_x = (n + geo_alt) * cos(geo_lat) * cos(geo_lon); //ECEF x
//	ecef_y = (n + geo_alt) * cos(geo_lat) * sin(geo_lon); //ECEF y
//	ecef_z = (n * (1 - e2) + geo_alt) * sin(geo_lat);     //ECEF z
//}

// Transformation from Lat Lon (Geodetic) to ECEF via WGS84 System
//latitude = (latitude) / (180.0) * M_PI;   // in radian
//longitude = (longitude) / (180.0) * M_PI; // in radian