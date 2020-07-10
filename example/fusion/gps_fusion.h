#pragma once

#include <string>
#include <chrono>
#include "eigen3/Eigen/Eigen"
#include "gps_parser.h"
#include "UTM.h"

using namespace std;
using namespace Eigen;

class gps_fusion {
private:


public:
    gps_fusion();

	//void caliberate_sensors(Eigen::Matrix4d cam_slam_pose, geo_location gps, float delta_time);

	static geo_location& start_gps;	// initial gps provided by user (GT)

	void init();

	bool initialized;
	bool isInitialized();

	//Eigen::Matrix<float, 3, 3> R_GPS; // Possible noise in the measured new GPS state (observation noise)
	//Eigen::Matrix<float, 3, 3> R_SLAM; // Possible noise in the measured new SLAM state (observation noise)

	unsigned long timestampLast;
	Eigen::Vector3f slamTranslationPrevious;
	Eigen::Quaternionf slamOrientation;

	/*
	Helper Functions
	*/

	unsigned long getCurrentTimestamp();
	unsigned long getLastTimestamp();
	void setLastTimestampToCurrent();
	double getDeltaTimeInSeconds();
	

	//// GPS WGS-84  <----> ECEF
	//const double a = 6378137.0; //WGS-84 semi-major axis
	//const double e2 = 6.6943799901377997e-3; //WGS-84 first eccentricity squared
	//const double a1 = 4.2697672707157535e+4; //a1 = a*e2
	//const double a2 = 1.8230912546075455e+9; //a2 = a1*a1
	//const double a3 = 1.4291722289812413e+2; //a3 = a1*e2/2
	//const double a4 = 4.5577281365188637e+9; //a4 = 2.5*a2
	//const double a5 = 4.2840589930055659e+4; //a5 = a1+a3
	//const double a6 = 9.9330562000986220e-1; //a6 = 1-e2

	//void ecef_to_geo(double& ecef_x, double& ecef_y, double& ecef_z, double& geo_lat, double& geo_lon, double& geo_alt);
	//void geo_to_ecef(double& geo_lat, double& geo_lon, double& geo_alt, double& ecef_x, double& ecef_y, double& ecef_z);
};