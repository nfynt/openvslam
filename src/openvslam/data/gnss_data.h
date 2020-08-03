#ifndef OPENVSLAM_GNSS_DATA_H
#define OPENVSLAM_GNSS_DATA_H

#include <string>

#include "Eigen/Eigen"

using namespace std;

namespace openvslam {
namespace gnss {

struct data {

    Eigen::Vector3d t_wgnss;

	//utm easting
    double utmx;
	//utm northing
    double utmy;
	//gnss range uncertainity in meter
    double uncertainity;
	//timestamp in millisecond since start of process
    long timestamp;

	data(double east, double north, double error, long time)
        : utmx(east), utmy(north), uncertainity(error), timestamp(time) {
        t_wgnss << east, 0, north;
	}

	data(Eigen::Vector3d trans, double error, long time)
        : t_wgnss(trans), uncertainity(error), timestamp(time) {
        utmx = trans(0, 0);
        utmy = trans(2, 0);
    }

	data() {
        utmx = utmy = uncertainity = 0.0;
        timestamp = 0;
        t_wgnss = Eigen::Vector3d::Identity();
	}
};


} // namespace gnss
} // namespace openvslam

#endif // OPENVSLAM_GPS_DATA_H
