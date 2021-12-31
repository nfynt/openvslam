#ifndef OPENVSLAM_GNSS_MODULE_H
#define OPENVSLAM_GNSS_MODULE_H

#include "openvslam/camera/base.h"
#include "openvslam/module/local_map_cleaner.h"
#include "openvslam/optimize/local_bundle_adjuster.h"

#include <memory>
#include <utility>

namespace openvslam {

class gnss_container {
public:
    gnss_container() { curr_timestamp = 0; }
    ~gnss_container() {}

    // gnss measurement queue
    std::list<std::pair<Eigen::Vector3d*, double*>> gnss_queue_;
    std::list<long*> gnss_timestamps_;

    // add world gnss measurement with variance factor to queue
    void enqueue_gnss_measurement(Eigen::Vector3d* t_wgnss, double* var_gps, long* timestamp) {
        gnss_queue_.push_back(std::make_pair(t_wgnss, var_gps));
        gnss_timestamps_.push_back(timestamp);
    }

    std::pair<Eigen::Vector3d*, double*> dequeue_gnss_measurement() {

        std::pair<Eigen::Vector3d*, double*> gnss = gnss_queue_.front();
        curr_timestamp = *gnss_timestamps_.front();
        gnss_queue_.pop_front();
        gnss_timestamps_.pop_front();

        return gnss;
    }

    // get gnss measurement close to provided timestamp
    std::pair<Eigen::Vector3d*, double*> dequeue_gnss_measurement(long timestamp) {
        std::pair<Eigen::Vector3d*, double*> gnss = gnss_queue_.front();
        curr_timestamp = *gnss_timestamps_.front();
        gnss_queue_.pop_front();
        gnss_timestamps_.pop_front();

        while (gnss_timestamps_.size() > 0 && *gnss_timestamps_.front() < timestamp) {
            gnss = gnss_queue_.front();
            curr_timestamp = *gnss_timestamps_.front();
            gnss_queue_.pop_front();
            gnss_timestamps_.pop_front();
        }

        return gnss;
    }

	// get the latest gnss measurement discarding all the previous measurement
	// useful when system_time is not in sync during video slam
    std::pair<Eigen::Vector3d*, double*> dequeue_latest_gnss() {
        
		std::pair<Eigen::Vector3d*, double*> gnss = gnss_queue_.back();
        curr_timestamp = *gnss_timestamps_.back();
        gnss_queue_.clear();
        gnss_timestamps_.clear();

        return gnss;
	}

    // get the timestamp of last dequeued gnss measurement
    long last_deq_timestamp() {
        return curr_timestamp;
    }

    // number of gnss measurement in queue
    inline int get_num_gnss_measurement() {
        return gnss_timestamps_.size();
    }

private:
    long curr_timestamp;
};

} // namespace openvslam

#endif // OPENVSLAM_GNSS_MODULE_H




/*
 __  _ _____   ____  _ _____  
|  \| | __\ `v' /  \| |_   _| 
| | ' | _| `. .'| | ' | | |   
|_|\__|_|   !_! |_|\__| |_|
 
*/