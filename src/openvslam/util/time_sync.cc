#include "time_sync.h"


namespace openvslam {
namespace util {

time_sync::time_sync() {
    this->video_timestamp = this->gps_timestamp = milliseconds(0);
    auto now_ms = time_point_cast<chrono::milliseconds>(chrono::system_clock::now());
    this->process_start_timestamp = now_ms.time_since_epoch().count();
    //std::cout << this->process_start_timestamp << std::endl;
}

milliseconds time_sync::is_gps_caught_up_video() {
    if (time_sync::gps_timestamp > time_sync::video_timestamp)
        return time_sync::gps_timestamp - time_sync::video_timestamp;
    else
        return time_sync::video_timestamp - time_sync::gps_timestamp;
}

milliseconds time_sync::is_video_caught_up_gps() {
    if (time_sync::gps_timestamp < time_sync::video_timestamp)
        return time_sync::video_timestamp - time_sync::gps_timestamp;
    else
        return time_sync::gps_timestamp - time_sync::video_timestamp;
}


long long time_sync::get_dt_start() {
    auto now_ms = time_point_cast<chrono::milliseconds>(chrono::system_clock::now());
    //std::cout << "t_s dt= " << (now_ms.time_since_epoch().count() - this->process_start_timestamp);
    return (now_ms.time_since_epoch().count() - this->process_start_timestamp);
}

} // namespace util
} // namespace openvslam