
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

#include "openvslam/util/time_sync.h"

#include "gps_network.h"
#include "gps_parser.h"
#include "gps_fusion.h"

using namespace std;

std::thread slam_th;
std::thread gps_th;
std::ofstream gps_out;
// current camera pose in cw: world -> camera (in SLAM CS)
openvslam::Mat44_t camera_pose;
// transformed gnss measurement GNSS -> slam world
Eigen::Vector3d t_wgnss;
gps_parser* gps;
// measured gnss UTM (universal time mercator)
geo_utm* curr_geo;
// last gnss utm
//geo_utm* last_geo;
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
//double var_gnss;
long last_gba_frame_cnt;

// test to check if file exists
inline bool exists(const std::string& name) {
    return std::experimental::filesystem::exists(name);
}

bool init_gnss_vslam_transformation();
//gnss init callback
void request_gnss_init() {
    if (init_gnss_vslam_transformation())
        spdlog::info("\n\nGNSS->VSLAM transformation success");
    else
        spdlog::info("\n\nGNSS->VSLAM transformation not successful");
}

// SLAM run in parallel thread
void run_slam(const std::string& vocab_path, const std::shared_ptr<openvslam::config>& cam_cfg,
              const std::string& vid_path, const std::string& map_db_path, const bool mapping) {
    cap = cv::VideoCapture(vid_path);

    if (!cap.isOpened()) {
        spdlog::critical("Unable to open video from path @ " + vid_path);
        slam_gps_running = false;
        return;
    }

    const auto frame_cnt = cap.get(CV_CAP_PROP_FRAME_COUNT);
    // ideal time for per frame ms
    //const milliseconds ideal_timestep_ms((long long)(1000 / cap.get(CV_CAP_PROP_FPS)));

    //total video time in millisecond
    //const double video_time_ms = (frame_cnt / cap.get(CV_CAP_PROP_FPS)) * 1000.0;

    spdlog::info("video frame count: " + to_string(frame_cnt));

    // build a SLAM system
    // openvslam::system SLAM(cam_cfg, vocab_path);
    SLAM = new openvslam::system(cam_cfg, vocab_path);

    //set time_sync
    SLAM->set_time_sync_ptr(time_s);

    SLAM->set_gps_data_is_used();

    //set gnss init callback
    SLAM->add_gnss_init_callback(&request_gnss_init);

    // startup the SLAM process with mapping settings
    if (map_db_path.empty() || !exists(map_db_path)) {
        spdlog::info("map_db not found or doesn't exist... will create new");
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

    //Disable loop closing
    SLAM->disable_loop_detector();
    spdlog::info("Disabled loop detecting for GNSS fusion");

    // create a viewer object
    // and pass the frame_publisher and the map_publisher
#ifdef USE_PANGOLIN_VIEWER
    pangolin_viewer::viewer viewer(cam_cfg, SLAM, SLAM->get_frame_publisher(), SLAM->get_map_publisher());
#endif

    while (!gps->is_valid) {
        spdlog::info("waiting for valid gps input...");
        std::this_thread::sleep_for(std::chrono::microseconds(1000));
    }

    double timestamp = 0.0;
    std::vector<double> track_times;
    track_times.reserve(frame_cnt);
    spdlog::info("SLAM starting");
    cv::Mat img;
    chrono::milliseconds wait_for_gps_thread(0);
    auto tp_1 = std::chrono::steady_clock::now();
    // run the SLAM in another thread
    std::thread thread([&]() {
        for (unsigned int i = 0; i < frame_cnt; ++i) {
            cap.read(img);

            if (!img.empty()) {
                gps->get_gps_value(curr_geo, timestamp * 1000);
                if (gps_initialized)
                    t_wgnss = R_wgnss * (curr_geo->get_vector() - start_geo->get_vector());
                else
                    t_wgnss = (curr_geo->get_vector() - start_geo->get_vector());

                SLAM->feed_GNSS_measurement(t_wgnss, curr_geo->uncertainity, timestamp * 1000);

                // input the current frame and estimate the camera pose
                camera_pose = SLAM->feed_monocular_frame(img, timestamp);
            }

            slam_tracking = SLAM->is_tracking();

            timestamp += 1.0 / cam_cfg->camera_->fps_;
            time_s->video_timestamp = milliseconds((long)(timestamp * 1000));

            const auto tp_2 = std::chrono::steady_clock::now();
            const auto track_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();

            track_times.push_back(track_time);

            wait_for_gps_thread = time_s->is_video_caught_up_gps();
            if (wait_for_gps_thread > chrono::milliseconds(10)) {
                // spdlog::info("waiting for gps thread:" + to_string(wait_for_gps_thread.count()));
                std::this_thread::sleep_for(wait_for_gps_thread);
            }

            // check if the termination of SLAM system is requested or not
            if (SLAM->terminate_is_requested() || !slam_gps_running) {
                slam_gps_running = false;
                break;
            }
            /*while (SLAM->is_local_ba_running()) {
                std::this_thread::sleep_for(std::chrono::microseconds(5));
            }
            if (SLAM->is_gps_initialized()) {
                const unsigned int cur_nr_kfs = SLAM.get_current_nr_kfs();
                if ((cur_nr_kfs - num_kfs_last_global_optim) > 30) {
                    num_kfs_last_global_optim = cur_nr_kfs;
                    SLAM->request_global_GPS_optim();
                    std::this_thread::sleep_for(std::chrono::microseconds(10));
                }
            }*/
            // wait until the global GPS BA is finished
            while (SLAM->global_GPS_optim_is_running()) {
                std::this_thread::sleep_for(std::chrono::microseconds(250));
            }
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
        //SLAM->save_map_database(map_db_path);
        spdlog::info("Skipping the saving of map-db");
    }
    spdlog::info("exiting slam thread");

    std::sort(track_times.begin(), track_times.end());
    const auto total_track_time = std::accumulate(track_times.begin(), track_times.end(), 0.0);
    std::cout << "median tracking time: " << track_times.at(track_times.size() / 2) << "[s]" << std::endl;
    std::cout << "mean tracking time: " << total_track_time / track_times.size() << "[s]" << std::endl;
}

// Launch GPS parsing thread
void run_gps() {
    spdlog::info("Starting gps parser");

    gps->start_reading(time_s);

    spdlog::info("gps parsing thread stopped");
    slam_gps_running = false;
}

bool init_gnss_vslam_transformation() {
    //Estimate the camera position in SLAM world cs by inverse tranformation
    Eigen::Vector3d w_pos = -camera_pose.block(0, 0, 3, 3).transpose() * camera_pose.block(0, 3, 3, 1);

    w_pos(1, 0) = 0; //ignore y position: assumed to be in same direction as UTM alt

    Eigen::Vector3d gnss_pos = gps_parser::get_direction_vector(*start_geo, *curr_geo);
    spdlog::info("intializing GNSS transform\ngps dist: {}",
                 curr_geo->sq_distance_from(start_geo));
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
        return false;
    }

    Eigen::Quaternion<double> t_quat(Eigen::AngleAxisd(angle, rotation_axis));
    t_quat.normalize();
    R_wgnss = t_quat.toRotationMatrix();

    std::cout << "Rotation matrix (world->gps)\n"
              << R_wgnss << "\n rotation angle: " << angle << "\n";

    gps_initialized = true;
    SLAM->set_gps_initialized(R_wgnss);

    //last_geo->copy_from(curr_geo);

    SLAM->request_global_GPS_optim();
    last_gba_frame_cnt = SLAM->get_current_nr_kfs();

    return true;
}

//Launch GPS SLAM fusion thread - outputs the correction gps
//updates the curr_geo to be used in SLAM thread
void fuse_gps_slam(const string& crr_gps_path, int freq = 1) {
    gps_out = std::ofstream(crr_gps_path, std::ios::out);
    gps_out << "# <lat,lon,alt,roll,pitch,yaw>\n";
    gps_out.close();

    gps_initialized = false;
    //wait for gps and slam thread to start
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));

    const long long iter_time = 1000 / freq;

    spdlog::info("Synchronizing coordinate system...");

    //estimate transformation between SLAM world and GPS based on distance
    while (!gps_initialized && slam_gps_running) {
        //if (curr_geo->sq_distance_from(start_geo) > 225.0 || SLAM->get_current_nr_kfs() > 30) {
        //    //the sensor has roughly moved 15 meters from start or 30 keyframes have been added
        //    //- good enough to initialize?

        //    if (init_gnss_vslam_transformation())
        //        break;
        //    else
        //        continue;
        //}
        //else {
        //    //spdlog::info("GPS dist: {}\t curr_gps: {}", curr_geo->sq_distance_from(start_geo),curr_geo->value());
        //}
        //check every 2 sec
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }

    while (slam_gps_running) {
        if (SLAM->tracker_is_paused()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<unsigned int>(1000)));
			continue;
        }

        auto tp_1 = std::chrono::steady_clock::now();
        if (slam_tracking) {
            const unsigned int curr_nr_kfs = SLAM->get_current_nr_kfs();
            //gps->update_gps_value(curr_geo);
            //!curr_geo->equals(last_geo) &&
            //performs global gps BA after every 30 new keyframes
            if (curr_nr_kfs - last_gba_frame_cnt >= 30) {
                spdlog::info("requesting GPS GBA");
                SLAM->request_global_GPS_optim();
                last_gba_frame_cnt = curr_nr_kfs;
            }

            //Get camera position in SLAM world and convert to GNSS CS
            Eigen::Vector3d pos_gnss = R_wgnss.transpose() * (-camera_pose.block(0, 0, 3, 3).transpose() * camera_pose.block(0, 3, 3, 1));

            //Get attitude information
            Eigen::Quaterniond rot_q(camera_pose.block<3, 3>(0, 0));
            auto rot_qn = R_wgnss.transpose() * rot_q.toRotationMatrix();
            auto euler = rot_qn.eulerAngles(0, 1, 2);

            spdlog::info("UTM sq distance: {}\nt_wgnss: {}\ngnss pose: {}\nvid_time: {}\tgnss_time: {}",
                         curr_geo->sq_distance_from(start_geo), t_wgnss.transpose(), pos_gnss.transpose(), time_s->video_timestamp.count(), time_s->gps_timestamp.count());

            pos_gnss += Eigen::Vector3d(start_geo->x, 0, -start_geo->y);
            geo_utm loc(pos_gnss.x(), pos_gnss.z(), curr_geo->zone);

            gps_out = std::ofstream(crr_gps_path, std::ios::app);
            //gps_out << loc.convert_utm_to_gps(curr_geo->altitude).value() << "," << euler(0) << "," << euler(1) << "," << euler(2) << "\n";
            gps_out << curr_geo->convert_utm_to_gps(curr_geo->altitude).value() << ","
                    << loc.convert_utm_to_gps(curr_geo->altitude).value() << ","
                    << euler(0) << "," << euler(1) << "," << euler(2) << "\n";
            gps_out.close();
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
    auto video_path = op.add<popl::Value<std::string>>("i", "video", "input video path");
    auto gps_path = op.add<popl::Value<std::string>>("g", "gps", "input gps txt path");
    auto crr_gps_path = op.add<popl::Value<std::string>>("o", "cgps", "corrected gps txt path");
    auto st_lat = op.add<popl::Value<double>>("l", "lat", "Start latitude", 0.0);
    auto st_lon = op.add<popl::Value<double>>("m", "lon", "Start longitude", 0.0);
    auto st_alt = op.add<popl::Value<double>>("a", "alt", "Start altitude", 0.0);
    //auto gnss_var = op.add<popl::Value<double>>("z", "var", "GNSS variance", 0.000010);
    auto map_db_path = op.add<popl::Value<std::string>>("p", "map-db", "store a map database at this path after SLAM", "");
    auto mapping = op.add<popl::Value<bool>>("n", "map-n", "enable mapping of existing map-db", false);
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

    // check validness of options
    if (help->is_set()) {
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }

    if (!video_path->is_set() || !gps_path->is_set() || !crr_gps_path->is_set() || !vocab_file_path->is_set() || !config_file_path->is_set()) {
        std::cerr << "invalid arguments" << std::endl;
        std::cerr << std::endl;
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }

    //var_gnss = gnss_var->value();
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

    // initialize params and start threads

    time_s = new openvslam::util::time_sync();
    gps = new gps_parser(gps_path->value());
    gps_th = thread(run_gps);

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    //set start gps location
    if (st_lat->value() != 0) {
        geo_location wgs = geo_location(st_lat->value(), st_lon->value(), st_alt->value());
        start_geo = new geo_utm(wgs);
        curr_geo = new geo_utm(*start_geo);
        //last_geo = new geo_utm(*curr_geo);
        spdlog::info("start gps(wgs84)\t{}\nUTM\t{}", wgs.value(), start_geo->value());
    }
    else {
        //utm start,curr and last init
        if (gps->is_valid) {
            if (start_geo == nullptr) {
                start_geo = new geo_utm();
                curr_geo = new geo_utm();
                // last_geo = new geo_utm();
                gps->update_gps_value(start_geo);
                curr_geo->copy_from(start_geo);
                //last_geo->copy_from(start_geo);
            }
            else {
                curr_geo->copy_from(start_geo);
                //last_geo->copy_from(curr_geo);
            }
            spdlog::info("start gps utm - {}", start_geo->value());
            spdlog::info("curr gps utm - {}", curr_geo->value());
        }
        else {
            spdlog::critical("fusion: invalid gps parser");
        }
    }
    //start slam and gps processes
    slam_gps_running = true;

    slam_th = thread(run_slam, vocab_file_path->value(), cfg, video_path->value(), map_db_path->value(), mapping->value());

    fuse_gps_slam(crr_gps_path->value());

    gps_th.join();
    slam_th.join();

    spdlog::info("Exiting process");

    return EXIT_SUCCESS;
}

/*
 __  _ _____   ____  _ _____  
|  \| | __\ `v' /  \| |_   _| 
| | ' | _| `. .'| | ' | | |   
|_|\__|_|   !_! |_|\__| |_|
 
*/