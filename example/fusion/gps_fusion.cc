#include "gps_fusion.h"

using namespace Eigen;

// State:
// x: World position in X
// y: World position in Y
// z: World position in Z
// vx: Velocity in X
// vy: Velocity in Y
// vz: Velocity in Z

gps_fusion::gps_fusion() {
    this->initialized = false;
    this->timestampLast = getCurrentTimestamp();
}

//Eigen::MatrixX3d cam_trans; //Nx3 matrix of camera positions for calibration (covariance estimate)
//Eigen::MatrixX3d gps_pos;	//Nx3 matrix of gps positions for calibration (covariance estimate)
//// Use first 5 seconds for estimating initial params. delta_time is the update interval
//void caliberate_sensors(Eigen::Vector3d translation, geo_location gps, float delta_time) {
//    //R_GPS_sigma (3x3)
//	//R_SLAM_sigma (3x3)
//
//	Eigen::MatrixXd centered = cam_trans.rowwise() - cam_trans.colwise().mean();
//    Eigen::MatrixXd cam_trans_cov = (centered.adjoint() * centered) / double(cam_trans.rows() - 1);
//}

bool gps_fusion::isInitialized() {
    return this->initialized;
}

void gps_fusion::init(float initx, float inity, float initz,
                      float initvx, float initvy, float initvz,
                      float sigmax, float sigmay, float sigmaz,
                      float sigmavx, float sigmavy, float sigmavz,
                      float qx, float qy, float qz,
                      float qvx, float qvy, float qvz,
                      float rx, float ry, float rz,
                      float rvx, float rvy, float rvz) {
    this->initx = initx;
    this->inity = inity;
    this->initz = initz;
    this->initvx = initvx;
    this->initvy = initvy;
    this->initvz = initvz;
    this->sigmax = sigmax;
    this->sigmay = sigmay;
    this->sigmaz = sigmaz;
    this->sigmavx = sigmavx;
    this->sigmavy = sigmavy;
    this->sigmavz = sigmavz;
    this->qx = qx;
    this->qy = qy;
    this->qz = qz;
    this->qvx = qvx;
    this->qvy = qvy;
    this->qvz = qvz;
    this->rx = rx;
    this->ry = ry;
    this->rz = rz;
    this->rvx = rvx;
    this->rvy = rvy;
    this->rvz = rvz;

    state = Eigen::Matrix<float, 6, 1>::Zero();
    state(0) = initx;
    state(1) = inity;
    state(2) = initz;
    state(3) = initvx;
    state(4) = initvy;
    state(5) = initvz;
    sigma = Eigen::Matrix<float, 6, 6>::Zero();
    sigma(0, 0) = sigmax;
    sigma(1, 1) = sigmay;
    sigma(2, 2) = sigmaz;
    sigma(3, 3) = sigmavx;
    sigma(4, 4) = sigmavy;
    sigma(5, 5) = sigmavz;

    Q = Eigen::Matrix<float, 6, 6>::Zero();
    Q(0, 0) = qx;
    Q(1, 1) = qy;
    Q(2, 2) = qz;
    Q(3, 3) = qvx;
    Q(4, 4) = qvy;
    Q(5, 5) = qvz;
    R_GPS = Eigen::Matrix<float, 3, 3>::Zero();
    R_GPS(0, 0) = rx;
    R_GPS(1, 1) = ry;
    R_GPS(2, 2) = rz;
    R_SLAM = Eigen::Matrix<float, 3, 3>::Zero();
    R_SLAM(0, 0) = rvx;
    R_SLAM(1, 1) = rvy;
    R_SLAM(2, 2) = rvz;

    dsoTranslationPrevious = Eigen::Vector3f::Zero();
    dsoOrientation = Eigen::Quaternionf::Identity();

    this->initialized = true;
}

void gps_fusion::predict(float timestep) {
    if (!this->initialized)
        return;
    /*   
         This is the implementation based on a linear Kalman Filter and using the constant velocity model
         
         [x']  = [ 1   0   0  Timestep      0       0      ]  [x]
         [y']  = [ 0   1   0      0     Timestep    0      ]  [y]
         [z']  = [ 0   0   1      0         0     Timestep ]  [z]
         [vx'] = [ 0   0   0      1         0       0      ]  [vx]
         [vy'] = [ 0   0   0      0         1       0      ]  [vy]
         [vz'] = [ 0   0   0      0         0       1      ]  [vz]
         
     */

    // State transition matrix:
    Eigen::Matrix<float, 6, 6> Phi = Eigen::Matrix<float, 6, 6>::Zero();
    Phi(0, 0) = Phi(1, 1) = Phi(2, 2) = Phi(3, 3) = Phi(4, 4) = Phi(5, 5) = 1;
    Phi(0, 3) = Phi(1, 4) = Phi(2, 5) = timestep;

    //// Process noise matrix (recalulate based on timestep value):
    //float T = timestep;
    //float T2 = T * T;
    //float T3 = T2 * timestep;
    //Eigen::Matrix<float, 6, 6> Q_currentTimestep = Q; // Also called Q_k
    //Q_currentTimestep.Zero();

    ///*
    //    Qk = [sigma_w_v*T3/3 + sigma_w_x*T, sigma_w_v*T2/2,   0,                            0,                0,                            0                 ],
    //         [sigma_w_v*T2/2,               sigma_w_v*T,      0,                            0,                0,                            0                 ],
    //         [0,                            0,                sigma_w_v*T3/3 + sigma_w_v*T, sigma_w_v*T2/2,   0,                            0                 ],
    //         [0,                            0,                sigma_w_v*T2/2,               sigma_w_v*T,      0,                            0                 ],
    //         [0,                            0,                0,                            0,                sigma_w_v*T3/3 + sigma_w_v*T, sigma_w_v*T2/2    ],
    //         [0,                            0,                0,                            0,                sigma_w_v*T2/2,               sigma_w_v*T       ]
    // */

    //// Process noise for positions:
    //Q_currentTimestep(0, 0) = this->sigmavx * T3 / 3 + this->sigmax * T;
    //Q_currentTimestep(1, 1) = this->sigmavy * T3 / 3 + this->sigmay * T;
    //Q_currentTimestep(2, 2) = this->sigmavz * T3 / 3 + this->sigmaz * T;

    //// Process noise for velocities:
    //Q_currentTimestep(3, 3) = this->sigmavx * T;
    //Q_currentTimestep(4, 4) = this->sigmavy * T;
    //Q_currentTimestep(5, 5) = this->sigmavz * T;

    // Control matrix
    //Eigen::Matrix<float, 6, 6> B = Eigen::Matrix<float, 6, 6>::Zero();
    // Control vector
    //Eigen::Matrix<float, 6, 1> control = Eigen::Matrix<float, 6, 1>::Zero();

    // Predicted (a-priori) state estimate (vector)
    state = Phi * state;	//+B* control;

    // Predicted (a priori) estimated state covariance (matrix)
    sigma = Phi * sigma * Phi.transpose(); //+ Q_currentTimestep;
}

void gps_fusion::updateGPS(const Eigen::Vector3f& measurement) {
    if (!this->initialized)
        return;
    
    Eigen::Matrix<float, 3, 6> H_GPS;
    H_GPS << 1, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0;

    Eigen::Vector3f y = measurement - H_GPS * state; // Innovation

    Eigen::Matrix<float, 3, 3> s_inv = (H_GPS * (sigma * H_GPS.transpose()) + R_GPS).inverse();

    //Eigen::Matrix<float,6,2> K = ( sigma * H_GPS.transpose() ) * (H_GPS * (sigma * H_GPS.transpose()) + R).inverse();
    Eigen::Matrix<float, 6, 3> K = sigma * H_GPS.transpose() * s_inv; // Kalman Gain

    // Apply the calculated correction to the current state:
    state = (state + K * y);
    Eigen::Matrix<float, 6, 6> K0 = Eigen::Matrix<float, 6, 6>::Identity() - K * H_GPS;
    sigma = K0 * sigma;
}

void gps_fusion::updateSLAM(const Eigen::Vector3f& translation, const Eigen::Quaternionf orientation, float timestep) {
    if (!this->initialized)
        return;

    // Calculate velocity vector
    Eigen::Vector3f velocity = (slamTranslationPrevious - translation) / timestep;

    slamTranslationPrevious = translation;
    slamOrientation = orientation;

    Eigen::Matrix<float, 3, 6> H_SLAM;
    H_SLAM << 0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 1;

    // ?Add Orientation into state vector

    Eigen::Vector3f y = velocity - H_SLAM * state; // Innovation

    Eigen::Matrix<float, 3, 3> s_inv = (H_SLAM * (sigma * H_SLAM.transpose()) + R_SLAM).inverse();
	Eigen::Matrix<float, 6, 3> K = sigma * H_SLAM.transpose() * s_inv; // Kalman Gain

    // Apply the calculated correction to the current state:
    state = (state + K * y);
    Eigen::Matrix<float, 6, 6> K0 = Eigen::Matrix<float, 6, 6>::Identity() - K * H_SLAM;
    sigma = K0 * sigma;
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

void gps_fusion::getStateAsSE3(Eigen::Vector3d& translation, Eigen::Quaternionf& rotation) {
    translation = Eigen::Vector3d(state[0], state[1], state[2]);
    rotation = this->slamOrientation;
}

/*
     CONVERT to and from Geodetic (lat, lon, height) to ECEF coordinates (x, y, z)
     Ported to C++ from: http://danceswithcode.net/engineeringnotes/geodetic_to_ecef/geodetic_to_ecef.html
     
     */

//Convert Earth-Centered-Earth-Fixed (ECEF) to lat, Lon, Altitude
//Input is a three element array containing x, y, z in meters
//Returned array contains lat and lon in radians, and altitude in meters
void gps_fusion::ecef_to_geo(double& ecef_x, double& ecef_y, double& ecef_z, double& geo_lat, double& geo_lon, double& geo_alt) {
    double zp, w2, w, r2, r, s2, c2, u, v, s, ss, c, g, rg, rf, f, m, p;

    zp = fabs(ecef_z);
    w2 = ecef_x * ecef_x + ecef_y * ecef_y;
    w = sqrt(w2);
    r2 = w2 + ecef_z * ecef_z;
    r = sqrt(r2);
    geo_lon = atan2(ecef_y, ecef_x); //Lon (final)
    s2 = ecef_z * ecef_z / r2;
    c2 = w2 / r2;
    u = a2 / r;
    v = a3 - a4 / r;
    if (c2 > 0.3) {
        s = (zp / r) * (1.0 + c2 * (a1 + u + s2 * v) / r);
        geo_lat = asin(s); //Lat
        ss = s * s;
        c = sqrt(1.0 - ss);
    }
    else {
        c = (w / r) * (1.0 - s2 * (a5 - u - c2 * v) / r);
        geo_lat = acos(c); //Lat
        ss = 1.0 - c * c;
        s = sqrt(ss);
    }
    g = 1.0 - e2 * ss;
    rg = a / sqrt(g);
    rf = a6 * rg;
    u = w - rg * c;
    v = zp - rf * s;
    f = c * u + s * v;
    m = c * v - s * u;
    p = m / (rf / g + f);
    geo_lat = geo_lat + p;     //Lat
    geo_alt = f + m * p / 2.0; //Altitude
    if (ecef_z < 0.0) {
        geo_lat *= -1.0; //Lat
    }
}

//Convert Lat, Lon, Altitude to Earth-Centered-Earth-Fixed (ECEF)
//Input is a three element array containing lat, lon (rads) and alt (m)
//Returned array contains x, y, z in meters
void gps_fusion::geo_to_ecef(double& geo_lat, double& geo_lon, double& geo_alt, double& ecef_x, double& ecef_y, double& ecef_z) {
    double n = a / sqrt(1 - e2 * sin(geo_lat) * sin(geo_lat));
    ecef_x = (n + geo_alt) * cos(geo_lat) * cos(geo_lon); //ECEF x
    ecef_y = (n + geo_alt) * cos(geo_lat) * sin(geo_lon); //ECEF y
    ecef_z = (n * (1 - e2) + geo_alt) * sin(geo_lat);     //ECEF z
}

// Transformation from Lat Lon (Geodetic) to ECEF via WGS84 System
//latitude = (latitude) / (180.0) * M_PI;   // in radian
//longitude = (longitude) / (180.0) * M_PI; // in radian