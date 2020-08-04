#include "openvslam/data/keyframe.h"
#include "openvslam/data/landmark.h"
#include "openvslam/data/map_database.h"
#include "openvslam/optimize/global_gps_bundle_adjuster.h"
#include "openvslam/optimize/internal/landmark_vertex_container.h"
#include "openvslam/optimize/internal/se3/shot_vertex_container.h"
#include "openvslam/optimize/internal/se3/reproj_edge_wrapper.h"
#include "openvslam/optimize/internal/gnss_measurement_edge.h"
#include "openvslam/util/converter.h"

#include "openvslam/mapping_module.h"

#include <g2o/core/solver.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/sparse_optimizer_terminate_action.h>
#include <g2o/types/slam3d/parameter_se3_offset.h>

#include <iostream>

namespace openvslam {
namespace optimize {

global_gps_bundle_adjuster::global_gps_bundle_adjuster(data::map_database* map_db,
                                                       const unsigned int num_iter,
                                                       const unsigned int nr_kfs_to_optim,
                                                       const bool use_huber_kernel)
    : map_db_(map_db), num_iter_(num_iter), nr_kfs_to_optim_(nr_kfs_to_optim), use_huber_kernel_(use_huber_kernel) {}


void global_gps_bundle_adjuster::set_mapping_module(mapping_module* mapper)
{
    mapper_ = mapper;
}

void global_gps_bundle_adjuster::optimize(const unsigned int lead_keyfrm_id_in_global_BA,
                                          bool* const force_stop_flag) {
    
	if (!is_map_scale_initialized || update_to_new_scale)
	{
        if (!start_map_scale_initalization()) {
            std::cout << "doing global gps BA without/previous map scale initialization\n";
        }
        else {
            std::cout << "Global GPS BA with scaled map init\n";
        }
	}

	// 1. Get all keyframes and landmarks

    set_running();

    std::cout << "=====RUNNING GLOBAL GPS OPTIM=====\n";
    const auto keyfrms = map_db_->get_all_keyframes();
    const auto lms = map_db_->get_all_landmarks();
    std::vector<bool> is_optimized_lm(lms.size(), true);

    // sort keyframes according to id
    std::map<unsigned int, data::keyframe*> sorted_kfs;
    for (auto kf : keyfrms) {
        sorted_kfs.insert(std::pair<unsigned int, data::keyframe*>(kf->id_, kf));
    }
    std::vector<data::keyframe*> all_kfs_sorted;
    for (auto pair : sorted_kfs) {
        all_kfs_sorted.push_back(pair.second);
    }
    // get the last nr_kfs_to_optim_ keyframes from the map
    std::vector<data::keyframe*> local_kfs(
        all_kfs_sorted.end() - std::min(all_kfs_sorted.size(), nr_kfs_to_optim_), all_kfs_sorted.end());

    std::cout << "Local nr kfs in global gps BA: " << local_kfs.size() << std::endl;
    std::cout << "Total nr of kfs in global gps BA: " << keyfrms.size() << std::endl;

    // 2. optimizerを構築

    auto linear_solver = ::g2o::make_unique<::g2o::LinearSolverCSparse<::g2o::BlockSolver_6_3::PoseMatrixType>>();
    auto block_solver = ::g2o::make_unique<::g2o::BlockSolver_6_3>(std::move(linear_solver));
    auto algorithm = new ::g2o::OptimizationAlgorithmLevenberg(std::move(block_solver));

    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(algorithm);

    optimizer.setVerbose(true);

    if (force_stop_flag) {
        optimizer.setForceStopFlag(force_stop_flag);
    }

    // 3. keyframeをg2oのvertexに変換してoptimizerにセットする

    // shot vertexのcontainer
    internal::se3::shot_vertex_container keyfrm_vtx_container(0, keyfrms.size());

    // for gps priors
    g2o::ParameterSE3Offset* offset = new g2o::ParameterSE3Offset();
    offset->setId(0);
    optimizer.addParameter(offset);

    // keyframesをoptimizerにセット
    unsigned int fixed_id = local_kfs[0]->id_;
    for (const auto keyfrm : local_kfs) {
        if (!keyfrm) {
            continue;
        }
        if (keyfrm->will_be_erased()) {
            continue;
        }
        auto keyfrm_vtx = keyfrm_vtx_container.create_vertex(keyfrm, keyfrm->id_ == fixed_id);
        optimizer.addVertex(keyfrm_vtx);

        // add gps priors to test
        //if (keyfrm->has_gnss_measurement()) 
		{
            auto gps_edge = new internal::gnss_measurement_edge();
            Vec3_t obs = keyfrm->get_gnss_data().t_wgnss;
            gps_edge->setMeasurement(obs);
            Mat33_t info_mat = Mat33_t::Identity();
            info_mat(0, 0) = keyfrm->get_gnss_data().uncertainity / 10.0;
            info_mat(1, 1) = keyfrm->get_gnss_data().uncertainity / 100.0; //high variance for altitude
            info_mat(2, 2) = keyfrm->get_gnss_data().uncertainity / 10.0;
            gps_edge->setInformation(info_mat);
            gps_edge->setVertex(0, keyfrm_vtx);
            //gps_edge->setParameterId(0, 0);

            optimizer.addEdge(gps_edge);
        }
    }

    // 4. keyframeとlandmarkのvertexをreprojection edgeで接続する

    // landmark vertexのcontainer
    internal::landmark_vertex_container lm_vtx_container(keyfrm_vtx_container.get_max_vertex_id() + 1, lms.size());

    // reprojection edgeのcontainer
    using reproj_edge_wrapper = internal::se3::reproj_edge_wrapper<data::keyframe>;
    std::vector<reproj_edge_wrapper> reproj_edge_wraps;
    reproj_edge_wraps.reserve(10 * lms.size());

    // 有意水準5%のカイ2乗値
    // DOF=2
    constexpr float chi_sq_2D = 5.99146;
    const float sqrt_chi_sq_2D = std::sqrt(chi_sq_2D);
    // DOF=3
    constexpr float chi_sq_3D = 7.81473;
    const float sqrt_chi_sq_3D = std::sqrt(chi_sq_3D);

    for (unsigned int i = 0; i < lms.size(); ++i) {
        auto lm = lms.at(i);
        if (!lm) {
            continue;
        }
        if (lm->will_be_erased()) {
            continue;
        }
        // landmarkをg2oのvertexに変換してoptimizerにセットする
        auto lm_vtx = lm_vtx_container.create_vertex(lm, false);
        optimizer.addVertex(lm_vtx);

        unsigned int num_edges = 0;
        const auto observations = lm->get_observations();
        for (const auto& obs : observations) {
            auto keyfrm = obs.first;
            auto idx = obs.second;
            if (!keyfrm) {
                continue;
            }
            if (keyfrm->will_be_erased()) {
                continue;
            }

            if (!keyfrm_vtx_container.contain(keyfrm)) {
                continue;
            }

            const auto keyfrm_vtx = keyfrm_vtx_container.get_vertex(keyfrm);
            const auto& undist_keypt = keyfrm->undist_keypts_.at(idx);
            const float x_right = keyfrm->stereo_x_right_.at(idx);
            const float inv_sigma_sq = keyfrm->inv_level_sigma_sq_.at(undist_keypt.octave);
            const auto sqrt_chi_sq = (keyfrm->camera_->setup_type_ == camera::setup_type_t::Monocular)
                                         ? sqrt_chi_sq_2D
                                         : sqrt_chi_sq_3D;
            auto reproj_edge_wrap = reproj_edge_wrapper(keyfrm, keyfrm_vtx, lm, lm_vtx,
                                                        idx, undist_keypt.pt.x, undist_keypt.pt.y, x_right,
                                                        inv_sigma_sq, sqrt_chi_sq, use_huber_kernel_);
            reproj_edge_wraps.push_back(reproj_edge_wrap);
            optimizer.addEdge(reproj_edge_wrap.edge_);
            ++num_edges;
        }

        if (num_edges == 0) {
            optimizer.removeVertex(lm_vtx);
            is_optimized_lm.at(i) = false;
        }
    }

    // 5. Perform optimization
    ::g2o::SparseOptimizerTerminateAction* terminateAction = 0;
    terminateAction = new ::g2o::SparseOptimizerTerminateAction;
    terminateAction->setGainThreshold(1e-4);
    terminateAction->setMaxIterations(num_iter_);
    optimizer.addPostIterationAction(terminateAction);
    optimizer.initializeOptimization();
    optimizer.optimize(num_iter_);

    if (force_stop_flag && *force_stop_flag) {
        return;
    }

    // 6. Retrieve results

    for (auto keyfrm : local_kfs) {
        if (keyfrm->will_be_erased()) {
            continue;
        }
        auto keyfrm_vtx = keyfrm_vtx_container.get_vertex(keyfrm);
        const auto cam_pose_cw = util::converter::to_eigen_mat(keyfrm_vtx->estimate());
        if (lead_keyfrm_id_in_global_BA == 0) {
            keyfrm->set_cam_pose(cam_pose_cw);
        }
        else {
            keyfrm->cam_pose_cw_after_loop_BA_ = cam_pose_cw;
            keyfrm->loop_BA_identifier_ = lead_keyfrm_id_in_global_BA;
        }
    }

    for (unsigned int i = 0; i < lms.size(); ++i) {
        if (!is_optimized_lm.at(i)) {
            continue;
        }

        auto lm = lms.at(i);
        if (!lm) {
            continue;
        }
        if (lm->will_be_erased()) {
            continue;
        }

        auto lm_vtx = lm_vtx_container.get_vertex(lm);
        const Vec3_t pos_w = lm_vtx->estimate();

        if (lead_keyfrm_id_in_global_BA == 0) {
            lm->set_pos_in_world(pos_w);
            lm->update_normal_and_depth();
        }
        else {
            lm->pos_w_after_global_BA_ = pos_w;
            lm->loop_BA_identifier_ = lead_keyfrm_id_in_global_BA;
        }
    }
    std::cout << "=====FINISHED GLOBAL GPS OPTIM=====\n";
    // finished set is running to false SLAM can continue
    set_finished();
}

void mean_of_eigen_vec(const eigen_alloc_vector<Vec3_t>& in_vec,
                       Vec3_t& mean) {
    mean.setZero();
    for (size_t i = 0; i < in_vec.size(); ++i) {
        for (int j = 0; j < 3; ++j) {
            mean[j] += in_vec[i][j];
        }
    }
    const double nr_els = static_cast<double>(in_vec.size());
    for (int j = 0; j < 3; ++j) {
        mean[j] /= nr_els;
    }
}

bool global_gps_bundle_adjuster::start_map_scale_initalization(bool pause_mapper) {
    
    if (is_map_scale_initialized) {
        return true;
    }
	
    // loop all keyframes and start it
    auto kfs = map_db_->get_all_keyframes();
    eigen_alloc_vector<Vec3_t> gps_pos;
    //std::vector<double> sigma;
    eigen_alloc_vector<Vec3_t> cam_pos;

    for (auto kf : kfs) {
        const auto gps = kf->get_gnss_data();
        gps_pos.push_back(gps.t_wgnss);
        cam_pos.push_back(kf->get_cam_center());
    }

    // now calculate scale
    Vec3_t mean_gps;
    Vec3_t mean_cam;
    mean_of_eigen_vec(gps_pos, mean_gps);
    mean_of_eigen_vec(cam_pos, mean_cam);

    double sum_gps_diff = 0.0;
    double sum_cam_diff = 0.0;
    for (size_t i = 0; i < gps_pos.size(); ++i) {
        sum_gps_diff += (gps_pos[i] - mean_gps).squaredNorm();
        sum_cam_diff += (cam_pos[i] - mean_cam).squaredNorm();
    }
    const double scale = std::sqrt(sum_gps_diff / sum_cam_diff);
    const double diff_to_last = std::abs(last_scale_estimate_ - scale);
    double total_distance_traveled = 0.0;
    total_distance_traveled = (gps_pos[gps_pos.size() - 1] - gps_pos[0]).norm();
    std::cout << "mean_gps: " << mean_gps.transpose() << "\nmean_cam: " << mean_cam.transpose();
    std::cout << "Estimated scale: " << scale << std::endl;
    std::cout << "Diff to last estimate: " << diff_to_last << std::endl;
    std::cout << "distance_traveled: " << total_distance_traveled << "[m]" << std::endl;
    if (diff_to_last <= 1 && total_distance_traveled < min_traveled_distance_) {
        return false;
    }
    last_scale_estimate_ = scale;


    // scale map
    // stop all threads and scale the map
    {
        std::lock_guard<std::mutex> lock(mtx_thread_);
        gps_scaling_is_running_ = true;
    }

    // stop mapping module - should already be paused from glob_optim_module
    if (pause_mapper)
        mapper_->request_pause();

    while (!mapper_->is_paused() && !mapper_->is_terminated()) {
        std::cout << "waiting for mapping module to pause\n";
        std::this_thread::sleep_for(std::chrono::microseconds(50));
    }
    // lock the map
    std::lock_guard<std::mutex> lock2(data::map_database::mtx_database_);

    auto landmarks = map_db_->get_all_landmarks();
    for (auto lm : landmarks) {
        if (!lm) {
            continue;
        }
        lm->set_pos_in_world(lm->get_pos_in_world() * scale);
    }
    for (auto kf : kfs) {
        if (kf->id_ != 0) {
            Mat44_t cam_pose_cw = kf->get_cam_pose();
            cam_pose_cw.block<3, 1>(0, 3) *= scale;
            kf->set_cam_pose(cam_pose_cw);
        }
    }
    // finished scaling return to normal behaviour
    if (pause_mapper)
        mapper_->resume();

    gps_scaling_is_running_ = false;
    is_map_scale_initialized = true;
    std::cout << "scaled the map with gps measurements: "<<scale<<"\n";
    return true;
}

void global_gps_bundle_adjuster::set_running() {
    is_gba_running_ = true;
}

void global_gps_bundle_adjuster::set_finished() {
    is_gba_running_ = false;
}

bool global_gps_bundle_adjuster::is_running() {
    return is_gba_running_ || gps_scaling_is_running_;
}

void global_gps_bundle_adjuster::test_map_scale_factor() 
{
    //if (!is_map_scale_initialized) {
    //    start_map_scale_initalization(true);
    //    return;
    //}

    // loop all keyframes and start it
    auto kfs = map_db_->get_all_keyframes();
    eigen_alloc_vector<Vec3_t> gps_pos;
    //std::vector<double> sigma;
    eigen_alloc_vector<Vec3_t> cam_pos;

    for (auto kf : kfs) {
        const auto gps = kf->get_gnss_data();

        gps_pos.push_back(gps.t_wgnss);
        cam_pos.push_back(kf->get_cam_center());
    }

    // now calculate scale
    Vec3_t mean_gps;
    Vec3_t mean_cam;
    mean_of_eigen_vec(gps_pos, mean_gps);
    mean_of_eigen_vec(cam_pos, mean_cam);

    double sum_gps_diff = 0.0;
    double sum_cam_diff = 0.0;
    for (size_t i = 0; i < gps_pos.size(); ++i) {
        sum_gps_diff += (gps_pos[i] - mean_gps).squaredNorm();
        sum_cam_diff += (cam_pos[i] - mean_cam).squaredNorm();
    }
    const double scale = std::sqrt(sum_gps_diff / sum_cam_diff);
    const double diff_to_last = std::abs(last_scale_estimate_ - scale);
    std::cout << "Estimated scale: " << scale << std::endl;
    std::cout << "Diff to last estimate: " << diff_to_last << std::endl;
    if (diff_to_last <= 1) {
        update_to_new_scale= false;
        return;
    }

	update_to_new_scale = true;
}

} // namespace optimize
} // namespace openvslam
