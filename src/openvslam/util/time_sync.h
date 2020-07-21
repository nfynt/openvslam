#pragma once

#include <string>
#include <thread>
#include <chrono>

using namespace std;
using namespace chrono;

namespace openvslam {
namespace util {

class time_sync {
public:
    time_sync();

    // video timestamp since start - of the play of video (in ms)
    milliseconds video_timestamp;
    // gps timestamp since start - from the parsing process (in ms)
    milliseconds gps_timestamp;

	// Time diff of gps timestamp from video. return value >0.0 if gps is ahead of video.
    milliseconds is_gps_caught_up_video();

	// Time diff of video timestamp from gps. return >0.0 if video is ahead of gps.
    milliseconds is_video_caught_up_gps();

	// get milliseconds (dt) past since the start of this process - the dt is not in sync with system_time
    long long get_dt_start();


private:
	// local timestamp (in ms epoch) value at the start of GPS SLAM thread start
    long long process_start_timestamp;
};


} // namespace util
} // namespace openvslam



/*
 __  _ _____   ____  _ _____  
|  \| | __\ `v' /  \| |_   _| 
| | ' | _| `. .'| | ' | | |   
|_|\__|_|   !_! |_|\__| |_|
 
*/