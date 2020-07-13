
#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <fstream>

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

#include "gps_network.h"
#include "gps_parser.h"
#include "time_sync.h"
#include "gps_fusion.h"

using namespace std;

std::thread slam_th;
std::thread gps_th;
fstream gps_out;
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

time_sync* time_s;
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
// 3x3 rotation matrix to transform vector from SLAM world to GPS
Eigen::Matrix3d R_wgps;
openvslam::system* SLAM;
// expected gnss variance
float var_gnss;

// SLAM run in parallel thread
void run_slam(const std::string& vocab_path, const std::shared_ptr<openvslam::config>& cam_cfg,
              const std::string& vid_path, const std::string& map_db_path) {
    cap = cv::VideoCapture(vid_path);

    if (!cap.isOpened()) {
        spdlog::critical("Unable to open video from path @ " + vid_path);
        slam_gps_running = false;
        return;
    }

    const auto frame_cnt = cap.get(CV_CAP_PROP_FRAME_COUNT);
    // ideal time for per frame ms
    const milliseconds ideal_timestep_ms((long long)(1000 / cap.get(CV_CAP_PROP_FPS)));
    //total video time in millisecond
    const double video_time_ms = (frame_cnt / cap.get(CV_CAP_PROP_FPS)) * 1000.0;

    spdlog::info("video frame count: " + to_string(frame_cnt));

    // build a SLAM system
   // openvslam::system SLAM(cam_cfg, vocab_path);
    SLAM = new openvslam::system(cam_cfg, vocab_path);

    // startup the SLAM process
    SLAM->startup();

    // create a viewer object
    // and pass the frame_publisher and the map_publisher
#ifdef USE_PANGOLIN_VIEWER
    pangolin_viewer::viewer viewer(cam_cfg, SLAM, SLAM->get_frame_publisher(), SLAM->get_map_publisher());
#endif

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
            
			/*if (slam_reset_req) {
                spdlog::info("Requesting to reset slam on gps initialization");
                SLAM.request_reset();
                slam_reset_req = false;
                continue;
            }*/

            cap.read(img);

            if (!img.empty()) {
                // input the current frame and estimate the camera pose
                camera_pose = SLAM->feed_monocular_frame(img, timestamp);
            }

			slam_tracking = SLAM->is_tracking();

            timestamp += 1.0 / cam_cfg->camera_->fps_;
            time_s->video_timestamp += ideal_timestep_ms;

            const auto tp_2 = std::chrono::steady_clock::now();
            const auto track_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();

            track_times.push_back(track_time);

            wait_for_gps_thread = time_s->is_video_caught_up_gps();
            if (wait_for_gps_thread > chrono::milliseconds(10)) {
                //spdlog::info("waiting for gps thread:" + to_string(wait_for_gps_thread.count()));
                std::this_thread::sleep_for(wait_for_gps_thread - chrono::milliseconds((tp_2 - tp_1).count()));
            }

            // check if the termination of SLAM system is requested or not
            if (SLAM->terminate_is_requested() || !slam_gps_running) {
                break;
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
        viewer.run();
        spdlog::info("enabling 3D view");
    }
    else {
        //viewer.request_terminate();
        spdlog::info("running slam without 3D view");
    }

#endif //  USE_PANGOLIN_VIEWER

    thread.join();

    SLAM->shutdown();

    if (!map_db_path.empty()) {
        // output the map database
        SLAM->save_map_database(map_db_path);
    }
    spdlog::info("exiting slam thread");
    slam_gps_running = false;

    std::sort(track_times.begin(), track_times.end());
    const auto total_track_time = std::accumulate(track_times.begin(), track_times.end(), 0.0);
    std::cout << "median tracking time: " << track_times.at(track_times.size() / 2) << "[s]" << std::endl;
    std::cout << "mean tracking time: " << total_track_time / track_times.size() << "[s]" << std::endl;
}

// Launch GPS parsing thread
void run_gps(string gps_path) {
    gps = new gps_parser(gps_path);

    spdlog::info("Starting gps parser");

    gps->start_reading(*time_s);

    spdlog::info("gps parsing thread stopped");
    slam_gps_running = false;
}

//Launch GPS SLAM fusion thread - outputs the correction gps
//updates the curr_geo to be used in SLAM thread
void fuse_gps_slam(const string& crr_gps_path, int freq = 1) {
    
    //KF(camera_pose + curr_geo) -> corrected gps location
    gps_out.open(crr_gps_path, fstream::out);
    gps_initialized = false;
    const long long iter_time = 1000 / freq;
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
	//utm start,curr and last init
    if (gps->is_valid) {
        if (start_geo == nullptr) {
            start_geo = new geo_utm();
            gps->update_gps_value(*start_geo);

            curr_geo = new geo_utm(*start_geo);
            last_geo = new geo_utm(*curr_geo);
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
    
	//estimate transformation between SLAM world and GPS based on distance
    while (!gps_initialized && slam_gps_running) {
        gps->update_gps_value(*curr_geo);
        if (curr_geo->sq_distance_from(start_geo) > 5.0) {
            //the sensor has roughly moved 3 meters from start - good enough to initialize?

            //Estimate the camera position in SLAM world cs
            Eigen::Vector3d w_pos = camera_pose.block(0, 3, 3, 1);
            Eigen::Vector3d gps_pos = gps_parser::get_direction_vector(*start_geo, *curr_geo);
            spdlog::info("curr utm - {}", curr_geo->value());
            std::cout << "\nCamera pos " << w_pos.transpose() 
				<< "\nGPS pos -" << gps_pos.transpose() << std::endl;
            
			//w_pos.y = 0;	//ignore y position: assumed to be in same direction as UTM alt

            //Normalize the world and gps direction vector
            w_pos.normalize();
            gps_pos.normalize();

			spdlog::info("Coordinate Basis\nWorld:\n{}\nGPS{}", w_pos.transpose(), gps_pos.transpose());
            //Estimate the rotation matrix to transform from w_pos to gps_pos

            Eigen::Vector3d rotation_axis = gps_pos.cross(w_pos); //ideally will be aligned along Y-axis (UP) which is assumed to be commond between GPS and SLAM
            rotation_axis.normalize();
            double angle = std::acos(gps_pos.dot(w_pos));

            Eigen::Quaternion<double> t_quat(Eigen::AngleAxisd(angle, rotation_axis));
            t_quat.normalize();
            R_wgps = t_quat.toRotationMatrix();

            std::cout << "Rotation matrix (world->gps)\n"
                      << R_wgps << "\n";

            gps_initialized = true;
			//reset slam and update start position (GPS and SLAM) for continuous tracking
            //slam_reset_req = true;
            //start_geo->copy_from(curr_geo);
            last_geo->copy_from(curr_geo);
            break;
        }
        else {
            spdlog::info("GPS dist: {}\t curr_gps: {}", curr_geo->sq_distance_from(start_geo),curr_geo->value());
        }
        //check every 2 sec
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }

    while (slam_gps_running) {
        auto tp_1 = std::chrono::steady_clock::now();

        gps->update_gps_value(*curr_geo);

        if (curr_geo != last_geo && slam_tracking) {
			// gps translation transformed to slam world
            Eigen::Vector3d t_gnss = R_wgps.inverse() * (curr_geo->get_vector() - start_geo->get_vector());
            
			SLAM->feed_GNSS_measurement(t_gnss, var_gnss);

            spdlog::info("fusion:\ncurr_utm: {}\tt_gps: {}\ngps time: {}\tvid: {}",curr_geo->value(),t_gnss, time_s->gps_timestamp.count(), time_s->video_timestamp.count());
            //gps_out << curr_geo->value() + "\n";
            cout << "fusion cam: \n"
                 << camera_pose << endl;

            // camera position in SLAM world cs
            Eigen::Vector3d pos = camera_pose.block(0, 3, 3, 1);

            //Convert to GPS cs
            pos = R_wgps * pos;
            //add the start GPS position for offset
            pos += Eigen::Vector3d(start_geo->x, 0, start_geo->y);

			//pos: (lat, alt, lon) - change to gps_location formation!!
            gps_out << pos << "\n";
            spdlog::info("UTM distance: {}\tgps_pos(lat,alt,lon): {}", curr_geo->sq_distance_from(start_geo),pos.transpose());
            last_geo->copy_from(curr_geo);
        }

        const auto tp_2 = std::chrono::steady_clock::now();

        const auto track_time = std::chrono::duration_cast<milliseconds>(tp_2 - tp_1).count();
        const auto sleep_time = iter_time - track_time;

        if (sleep_time > 0.0) {
            spdlog::info("fusion thread sleep for: " + to_string(static_cast<unsigned int>(sleep_time)));
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
    auto gnss_var = op.add<popl::Value<float>>("z", "var", "GNSS variance", 20.0);
    auto map_db_path = op.add<popl::Value<std::string>>("p", "map-db", "store a map database at this path after SLAM", "");
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

    //set start gps location
    if (st_lat->value() != 0) {
        geo_location wgs = geo_location(st_lat->value(), st_lon->value(), st_alt->value());
        start_geo = new geo_utm(wgs);
        curr_geo = new geo_utm(*start_geo);
        last_geo = new geo_utm(*curr_geo);
        spdlog::info("start gnss\t(wgs84) {}\t(utm) {}", wgs.value(), start_geo->value());
        //gps_fusion::start_gps = *start_geo;
    }

    //start slam and gps processes
    slam_gps_running = true;
    //slam_reset_req = false;
    //time_sync::gps_timestamp = time_sync::video_timestamp = chrono::milliseconds(0);
    time_s = new time_sync();

    slam_th = thread(run_slam, vocab_file_path->value(), cfg, video_path->value(), map_db_path->value());
    gps_th = thread(run_gps, gps_path->value());

    fuse_gps_slam(crr_gps_path->value());

    //slam_th.join();
    //gps_th.join();
    spdlog::info("Exiting");

    return EXIT_SUCCESS;
}