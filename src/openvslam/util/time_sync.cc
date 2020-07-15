#include "time_sync.h"

namespace openvslam {
namespace util {


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


inline long long time_sync::get_dt_start() {
    auto now_ms = time_point_cast<chrono::milliseconds>(chrono::system_clock::now()).time_since_epoch().count();
    return (now_ms - this->process_start_timestamp);
}

} // namespace util
} // namespace openvslam