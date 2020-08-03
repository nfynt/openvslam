
#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <fstream>
#include <filesystem>

#ifdef USE_PANGOLIN_VIEWER
#include "pangolin_viewer/viewer.h"
#endif

#include <opencv2/core/core.hpp>
#include "opencv2/highgui.hpp"
#include "openvslam/system.h"
#include "openvslam/config.h"
#include <popl.hpp>
#include <spdlog/spdlog.h>
#include "spdlog/fmt/ostr.h"

#include "SFML/Network.hpp"

#include "openvslam/util/time_sync.h"

#include "gps_network.h"
#include "gps_parser.h"
#include "gps_fusion.h"

using namespace std;
using namespace cv;

std::thread slam_th;
std::thread gps_th;
std::ofstream gps_out;
// current camera pose in cw: world -> camera (in SLAM CS)
openvslam::Mat44_t camera_pose;
gps_parser* gps;
// measured gnss UTM (universal time mercator)
geo_utm* curr_geo;
// last gnss utm
geo_utm* last_geo;
// gps value in UTM converted from manually provided WGS84 by user at the start - static correction
geo_utm* start_geo;
// gps initialization - to estimation the transformation between GPS and SLAM coordinate system
bool gps_initialized;

// time synchronization between SLAM and GPS thread. Initialize it before SLAM::system
openvslam::util::time_sync* time_s;
// Video or camera capture
cv::VideoCapture cap;
// Flag to signal if both slam and gps are running. Exits the thread if either stops
bool slam_gps_running;
// use pangolin for frame and 3D visualization
bool enable_3d_view;
// request to reset slam on new initialization
//bool slam_reset_req;

// flag to check if slam is in tracking state
bool slam_tracking;
// 3x3 rotation matrix to transform vector from UTM -> SLAM world
// for translation the UTM start is assumed to same as first fixed frame of SLAM
Eigen::Matrix3d R_wgnss;
openvslam::system* SLAM;
// expected gnss variance
double var_gnss;
// string lat,long,alt
string msg;
sf::TcpSocket socket;
sf::Socket::Status status;
// video/gps update frequency set by the user. default = 20
int update_freq;



// test to check if file exists
inline bool exists(const std::string& name) {
    return std::experimental::filesystem::exists(name);
}



// SLAM run in parallel thread
void run_slam(const std::string& vocab_path, const std::shared_ptr<openvslam::config>& cam_cfg,
              const int& cam_ind, const int& width, const int& height, const std::string& map_db_path, const bool mapping) {
    
	cap = cv::VideoCapture(cam_ind);

    if (!cap.isOpened()) {
        spdlog::critical("Unable to capture cam index: {} ",cam_ind);
        slam_gps_running = false;
        return;
    }
    cap.set(CAP_PROP_FRAME_WIDTH, width);
    cap.set(CAP_PROP_FRAME_HEIGHT, height);
    cap.set(CAP_PROP_FPS, update_freq);

    // ideal time for per frame ms
    const milliseconds ideal_timestep_ms((long long)(1000.0 / update_freq));
    
    // build a SLAM system
    // openvslam::system SLAM(cam_cfg, vocab_path);
    SLAM = new openvslam::system(cam_cfg, vocab_path);

    //set time_sync
    SLAM->set_time_sync_ptr(time_s);

    // startup the SLAM process with mapping settings
    if (map_db_path.empty() || !exists(map_db_path)) {
        SLAM->startup();
    }
    else {
        SLAM->load_map_database(map_db_path);
        SLAM->startup(false);
        if (mapping)
            SLAM->enable_mapping_module();
        else
            SLAM->disable_mapping_module();
    }

    // create a viewer object
    // and pass the frame_publisher and the map_publisher
#ifdef USE_PANGOLIN_VIEWER
    pangolin_viewer::viewer viewer(cam_cfg, SLAM, SLAM->get_frame_publisher(), SLAM->get_map_publisher());
#endif

	//time stamp in seconds since start of this process
    double timestamp = 0.0;
    
	//track time in milliseconds
    long long track_time = 0;

    int frame_cnt=0;
    spdlog::info("SLAM starting");
    cv::Mat img;
    chrono::milliseconds wait_for_gps_thread(0);
    auto tp_1 = std::chrono::steady_clock::now();

    // run the SLAM in another thread
    std::thread thread([&]() {
        while(true) {

            cap.read(img);

            if (!img.empty()) {
                // input the current frame and estimate the camera pose
                camera_pose = SLAM->feed_monocular_frame(img, timestamp);
                frame_cnt++;
            }

            slam_tracking = SLAM->is_tracking();

            // check if the termination of SLAM system is requested or not
            if (SLAM->terminate_is_requested() || !slam_gps_running) {
                slam_gps_running = false;
                break;
            }

            auto tp_2 = std::chrono::steady_clock::now();
            long long delta_time = duration_cast<milliseconds>(tp_2 - tp_1).count();

            track_time += delta_time;
            timestamp += 1.0/cap.get(cv::CAP_PROP_FPS);
            time_s->video_timestamp += chrono::milliseconds(delta_time);


            tp_1 = tp_2;
        }

        // wait until the loop BA is finished
        while (SLAM->loop_BA_is_running()) {
            std::this_thread::sleep_for(std::chrono::microseconds(5000));
        }

// automatically close the viewer at the end
#ifdef USE_PANGOLIN_VIEWER
        if (enable_3d_view && !SLAM->terminate_is_requested())
            viewer.request_terminate();
#endif
    });

    // run the viewer in the current thread
#ifdef USE_PANGOLIN_VIEWER
    if (enable_3d_view) {
        spdlog::info("enabling 3D view");
        viewer.run();
    }
    else {
        //viewer.request_terminate();
        spdlog::info("running slam without 3D view");
    }

#endif //  USE_PANGOLIN_VIEWER

    thread.join();

    SLAM->shutdown();
    slam_gps_running = false;

    if (!map_db_path.empty()) {
        // output the map database
        SLAM->save_map_database(map_db_path);
    }
    spdlog::info("exiting slam thread");


//    std::cout << "median tracking time: " << track_times.at(track_times.size() / 2) << "[s]" << std::endl;
    std::cout << "mean tracking time: " << track_time / (frame_cnt * 1000) << "[s]" << std::endl;
}

// Launch GPS parsing thread
void run_gps(string ip, int port) {
    gps = new gps_parser(ip,port);

    spdlog::info("Starting gps parser");

    gps->connect_and_read(time_s);

    spdlog::info("gps parsing thread stopped");
    slam_gps_running = false;
}

//Launch GPS SLAM fusion thread - outputs the correction gps
//updates the curr_geo to be used in SLAM thread
void fuse_gps_slam(const string& crr_gps_path, int freq = 1) {
    //KF(camera_pose + curr_geo) -> corrected gps location
    gps_out = std::ofstream(crr_gps_path,std::ios::out);
    gps_out << "# <inp_lat,inp_lon,inp_alt,out_lat,out_lon,out_alt,roll,pitch,yaw>\n";

    gps_initialized = false;
    const long long iter_time = 1000 / freq;
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    //utm start,curr and last init
    if (gps->is_valid) {
        if (start_geo == nullptr) {
            start_geo = new geo_utm();
            curr_geo = new geo_utm();
            last_geo = new geo_utm();
            gps->update_gps_value(start_geo);
            curr_geo->copy_from(start_geo);
            last_geo->copy_from(start_geo);
        }
        else {
            curr_geo->copy_from(start_geo);
            last_geo->copy_from(curr_geo);
        }
        spdlog::info("start gps utm - {}", start_geo->value());
        spdlog::info("curr gps utm - {}", curr_geo->value());
    }
    else {
        spdlog::critical("fusion: invalid gps");
        slam_gps_running = false;
    }

    spdlog::info("Synchronizing coordinate system...");

    //estimate transformation between SLAM world and GPS based on distance
    while (!gps_initialized && slam_gps_running) {
        gps->update_gps_value(curr_geo);

        if (curr_geo->sq_distance_from(start_geo) > 400.0) {
            //the sensor has roughly moved 10 meters from start - good enough to initialize?

            //Estimate the camera position in SLAM world cs
            Eigen::Vector3d w_pos = camera_pose.block(0, 3, 3, 1);

            w_pos(1, 0) = 0; //ignore y position: assumed to be in same direction as UTM alt

            Eigen::Vector3d gnss_pos = gps_parser::get_direction_vector(*start_geo, *curr_geo);
            spdlog::info("curr utm - {}", curr_geo->value());
            std::cout << "\nCamera pos " << w_pos.transpose()
                      << "\nGPS pos " << gnss_pos.transpose() << std::endl;

            //Normalize the world and gps direction vector
            w_pos.normalize();
            gnss_pos.normalize();

            spdlog::info("Coordinate Basis\nWorld: {}\nGPS {}", w_pos.transpose(), gnss_pos.transpose());
            //Estimate the rotation matrix to transform from w_pos to gps_pos

            Eigen::Vector3d rotation_axis = gnss_pos.cross(w_pos); //ideally will be aligned along Y-axis (UP) which is assumed to be commond between GPS and SLAM
            rotation_axis.normalize();
            double angle = std::acos(gnss_pos.dot(w_pos));

            // degenrate case when vector a and b point's in opposite direction
            if (cos(angle) == -1) {
                spdlog::info("degenerate transformation case, will retry");
                continue;
            }

            Eigen::Quaternion<double> t_quat(Eigen::AngleAxisd(angle, rotation_axis));
            t_quat.normalize();
            R_wgnss = t_quat.toRotationMatrix();

            std::cout << "Rotation matrix (world->gps)\n"
                      << R_wgnss << "\n rotation angle: " << angle << "\n";

            gps_initialized = true;
            last_geo->copy_from(curr_geo);

            break;
        }
        else {
            //spdlog::info("GPS dist: {}\t curr_gps: {}", curr_geo->sq_distance_from(start_geo),curr_geo->value());
        }
        //check every 2 sec
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }

    while (slam_gps_running) {
        auto tp_1 = std::chrono::steady_clock::now();

        gps->update_gps_value(curr_geo);
        //!curr_geo->equals(last_geo) &&
        if (slam_tracking) {
            spdlog::info("inp gps: {}", curr_geo->convert_utm_to_gps(curr_geo->altitude).value());
            // utm coordinates transformed to slam world
            Eigen::Vector3d t_gnss = R_wgnss * (curr_geo->get_vector() - start_geo->get_vector());

            //verify this??
           // var_gnss = (gps->accumulated_delta_range + gps->speed) / ((gps->sat_count <= 0) ? 0.5 : gps->sat_count);

            SLAM->feed_GNSS_measurement(t_gnss, var_gnss, time_s->get_dt_start());

            Eigen::Vector3d pos = camera_pose.block(0, 3, 3, 1);
            spdlog::info("fusion:\ncurr_utm: {}\nt_gps: {}\nt_wslam: {}\ngps time: {}\tvid: {}",
                         curr_geo->value(), t_gnss.transpose(), pos.transpose(), time_s->gps_timestamp.count(), time_s->video_timestamp.count());

            // camera position in SLAM world cs
            Eigen::Quaterniond rot_q(camera_pose.block<3, 3>(0, 0));
            auto rot_qn = R_wgnss.transpose() * rot_q.toRotationMatrix();
            auto euler = rot_qn.eulerAngles(0, 1, 2);

            //Convert to GPS cs
            pos = R_wgnss.transpose() * pos; //transpose == inverse
            //add the start GPS position for offset
            pos += Eigen::Vector3d(start_geo->x, 0, start_geo->y);

            spdlog::info("UTM sq distance: {}\ncurr_gnss: {}\ncorrected gnss: {}",
                         curr_geo->sq_distance_from(start_geo), curr_geo->value(), pos.transpose());

            //convert to gps_location format!!

            geo_utm loc(pos.x(), pos.z(), curr_geo->zone);
            gps_out << curr_geo->convert_utm_to_gps(curr_geo->altitude).value() << ","
                    << loc.convert_utm_to_gps(curr_geo->altitude).value() << ","
                    << euler(0) << "," << euler(1) << "," << euler(2) << "\n";
            
            spdlog::info("corrected GPS\npos: {}\nheading: {}",
                         loc.convert_utm_to_gps().value(), euler.transpose());

            last_geo->copy_from(curr_geo);
        }

        const auto tp_2 = std::chrono::steady_clock::now();

        const auto track_time = std::chrono::duration_cast<milliseconds>(tp_2 - tp_1).count();
        const auto sleep_time = iter_time - track_time;

        if (sleep_time > 0.0) {
            //spdlog::info("fusion thread sleep for: " + to_string(static_cast<unsigned int>(sleep_time)));
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<unsigned int>(sleep_time)));
        }
    }

    gps->terminate_process();
    gps_out.close();
    spdlog::info("Finished gps slam fusion");
}

int main(int argc, char* argv[]) {
    // create options
    popl::OptionParser op("Allowed options for gps client:");
    auto help = op.add<popl::Switch>("h", "help", "produce help message");
    auto vocab_file_path = op.add<popl::Value<std::string>>("v", "vocab", "vocabulary file path");
    auto config_file_path = op.add<popl::Value<std::string>>("c", "config", "config file path");
    auto cam_ind = op.add<popl::Value<int>>("n", "cam", "camera index");
    auto width = op.add<popl::Value<int>>("x", "width", "video width", 640);
    auto height = op.add<popl::Value<int>>("y", "height", "video height", 480);
    auto ip = op.add<popl::Value<std::string>>("i", "ip", "IP address","localhost");
    auto port = op.add<popl::Value<int>>("p", "port", "Port address", 20175);
    auto freq = op.add<popl::Value<int>>("f", "freq", "update frequency for video and gps",20);
    auto crr_gps_path = op.add<popl::Value<std::string>>("o", "cgps", "corrected gps txt path","gps_corrected.txt");
    auto st_lat = op.add<popl::Value<double>>("l", "lat", "Start latitude", 0.0);
    auto st_lon = op.add<popl::Value<double>>("m", "lon", "Start longitude", 0.0);
    auto st_alt = op.add<popl::Value<double>>("a", "alt", "Start altitude", 0.0);
    auto gnss_var = op.add<popl::Value<double>>("z", "var", "GNSS variance", 10.0);
    auto map_db_path = op.add<popl::Value<std::string>>("d", "map-db", "store map database at this path after SLAM", "");
    auto mapping = op.add<popl::Value<bool>>("q", "map-n", "enable mapping of existing map-db", false);
    auto show_3d = op.add<popl::Value<int>>("s", "show", "show pangolin 3d view", 0);
    auto debug_mode = op.add<popl::Switch>("", "debug", "debug mode");

    try {
        op.parse(argc, argv);
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        std::cerr << std::endl;
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }
    std::cout << "popl done\n";
    // check validness of options
    if (help->is_set()) {
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }
    
    if (!vocab_file_path->is_set() || !config_file_path->is_set() || !cam_ind->is_set() || !crr_gps_path->is_set()) {
        std::cerr << "invalid arguments" << std::endl;
        std::cerr << std::endl;
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }
    std::cout << "params done\n";
    var_gnss = gnss_var->value();
    // setup logger
    spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] %^[%L] %v%$");
    if (debug_mode->is_set()) {
        spdlog::set_level(spdlog::level::debug);
    }
    else {
        spdlog::set_level(spdlog::level::info);
    }

    // load configuration
    std::shared_ptr<openvslam::config> cfg;
    try {
        cfg = std::make_shared<openvslam::config>(config_file_path->value());
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    if (show_3d->value() != 0)
        enable_3d_view = true;
    update_freq = freq->value();
    //set start gps location
    if (st_lat->value() != 0) {
        geo_location wgs = geo_location(st_lat->value(), st_lon->value(), st_alt->value());
        start_geo = new geo_utm(wgs);
        curr_geo = new geo_utm(*start_geo);
        last_geo = new geo_utm(*curr_geo);
        spdlog::info("start gps(wgs84)\t{}\nUTM\t{}", wgs.value(), start_geo->value());
        //gps_fusion::start_gps = *start_geo;
    }

    //start slam and gps processes
    slam_gps_running = true;
    //slam_reset_req = false;
    //time_sync::gps_timestamp = time_sync::video_timestamp = chrono::milliseconds(0);
    time_s = new openvslam::util::time_sync();

    slam_th = thread(run_slam, vocab_file_path->value(), cfg, cam_ind->value(), width->value(), height->value(),
		map_db_path->value(), mapping->value());
    gps_th = thread(run_gps, ip->value(),port->value());

    fuse_gps_slam(crr_gps_path->value());

    slam_th.join();
    gps_th.join();

    spdlog::info("Exiting process");

    return EXIT_SUCCESS;
}

/*
 __  _ _____   ____  _ _____  
|  \| | __\ `v' /  \| |_   _| 
| | ' | _| `. .'| | ' | | |   
|_|\__|_|   !_! |_|\__| |_|
 
*/