#ifndef OPENVSLAM_OPTIMIZE_GLOBAL_GPS_BUNDLE_ADJUSTER_H
#define OPENVSLAM_OPTIMIZE_GLOBAL_GPS_BUNDLE_ADJUSTER_H

namespace openvslam {

namespace data {
class map_database;
} // namespace data

class mapping_module;

namespace optimize {

class global_gps_bundle_adjuster {
public:
    /**
     * Constructor
     * @param map_db
     * @param num_iter
     * @param use_huber_kernel
     */
    explicit global_gps_bundle_adjuster(data::map_database* map_db,
                                        const unsigned int num_iter = 50,
                                        const unsigned int nr_kfs_to_optim = 100,
                                        const bool use_huber_kernel = true);

    /**
     * Destructor
     */
    virtual ~global_gps_bundle_adjuster() = default;

    /**
     * Perform optimization
     * @param lead_keyfrm_id_in_global_BA
     * @param force_stop_flag
     */
    void optimize(const unsigned int lead_keyfrm_id_in_global_BA = 0, bool* const force_stop_flag = nullptr);

    /**
     * Check if optim is currently running gor gbs of map-scale
     * @brief is_running
     * @return bool if it is running
     */
    bool is_running();

    //set mapping module
    void set_mapping_module(mapping_module* mapper);

    //check if map scale needs to be recalculated
    void test_map_scale_factor();

private:
    void set_running();
    void set_finished();

    //! mutex for access to pause procedure
    //mutable std::mutex mtx_thread_;

    //! map database
    const data::map_database* map_db_;

    //! mapping module
    mapping_module* mapper_;

    //! number of iterations of optimization
    unsigned int num_iter_;

    //! use Huber loss or not
    const bool use_huber_kernel_;

    //! how many keyframes we want to optimize from the current one
    size_t nr_kfs_to_optim_;

    //! if gba is running
    bool is_gba_running_ = false;

    //! if gps scaling is running
    bool gps_scaling_is_running_ = false;

    //! need to update scale
    bool update_to_new_scale = false;

    double last_scale_estimate_ = 0.0;

    //minimum travel distance for scale initialization in meters
    double min_traveled_distance_ = 10;

    //! if maps scale is initialized with gps
    bool is_map_scale_initialized = false;

    bool start_map_scale_initalization(bool pause_mapper = false);

    //! mutex for access to pause procedure
    mutable std::mutex mtx_thread_;
};

} // namespace optimize
} // namespace openvslam

#endif // OPENVSLAM_OPTIMIZE_GLOBAL_GPS_BUNDLE_ADJUSTER_H
