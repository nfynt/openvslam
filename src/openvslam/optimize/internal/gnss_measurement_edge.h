#ifndef OPENVSLAM_OPTIMIZER_G2O_GNSS_MEASUREMENT_EDGE_H
#define OPENVSLAM_OPTIMIZER_G2O_GNSS_MEASUREMENT_EDGE_H

#include "openvslam/type.h"
#include "openvslam/optimize/internal/gnss_vertex.h"
#include "openvslam/optimize/internal/se3/shot_vertex.h"

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>

namespace openvslam {
namespace optimize {
namespace internal {

class gnss_measurement_edge final : public g2o::BaseUnaryEdge<3, Vec3_t, gnss_vertex> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    gnss_measurement_edge();

    bool read(std::istream& is) override;

    bool write(std::ostream& os) const override;

    void computeError() override;

    void linearizeOplus() override;

    //g2o::OptimizableGraph::Edge* edge_;
};

inline gnss_measurement_edge::gnss_measurement_edge()
    : g2o::BaseUnaryEdge<3, Vec3_t, gnss_vertex>() {}

inline bool gnss_measurement_edge::read(std::istream& is) {
    for (unsigned int i = 0; i < 3; ++i) {
        is >> _measurement(i);
    }
    for (unsigned int i = 0; i < 3; ++i) {
        for (unsigned int j = i; j < 3; ++j) {
            is >> information()(i, j);
            if (i != j) {
                information()(j, i) = information()(i, j);
            }
        }
    }
    return true;
}

inline bool gnss_measurement_edge::write(std::ostream& os) const {
    for (unsigned int i = 0; i < 3; ++i) {
        os << measurement()(i) << " ";
    }
    for (unsigned int i = 0; i < 3; ++i) {
        for (unsigned int j = i; j < 3; ++j) {
            os << " " << information()(i, j);
        }
    }
    return os.good();
}

inline void gnss_measurement_edge::computeError() {
    const auto v1 = static_cast<const gnss_vertex*>(_vertices.at(0));
    const Vec3_t obs(_measurement);
	//t_gnss - t_wslam
    _error = obs - v1->estimate();
}

inline void gnss_measurement_edge::linearizeOplus() {
    auto vi = static_cast<gnss_vertex*>(_vertices.at(0));
    const Vec3_t pos_c = vi->estimate();

    const auto x = pos_c(0);
    const auto y = pos_c(1);
    const auto z = pos_c(2);
    const auto z_sq = z * z;

	//e = [(x2-x1) (y2-y1) (z2-z1)]
	//de/dx
    _jacobianOplusXi(0, 0) = 1;
    _jacobianOplusXi(0, 1) = 0;
    _jacobianOplusXi(0, 2) = 0;
    _jacobianOplusXi(0, 3) = 0;
    _jacobianOplusXi(0, 4) = 0;
    _jacobianOplusXi(0, 5) = 0;
	//de/dy
    _jacobianOplusXi(1, 0) = 0;
    _jacobianOplusXi(1, 1) = 1;
    _jacobianOplusXi(1, 2) = 0;
    _jacobianOplusXi(1, 3) = 0;
    _jacobianOplusXi(1, 4) = 0;
    _jacobianOplusXi(1, 5) = 0;
	//de/dz
    _jacobianOplusXi(2, 0) = 0;
    _jacobianOplusXi(2, 1) = 0;
    _jacobianOplusXi(2, 2) = 1;
    _jacobianOplusXi(2, 3) = 0;
    _jacobianOplusXi(2, 4) = 0;
    _jacobianOplusXi(2, 5) = 0;
}



} // namespace internal
} // namespace optimize
} // namespace openvslam

#endif // OPENVSLAM_OPTIMIZER_G2O_GNSS_MEASUREMENT_EDGE_H
