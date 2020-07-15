#ifndef OPENVSLAM_OPTIMIZER_G2O_GNSS_VERTEX_H
#define OPENVSLAM_OPTIMIZER_G2O_GNSS_VERTEX_H

#include "openvslam/type.h"

#include <g2o/core/base_vertex.h>

namespace openvslam {
namespace optimize {
namespace internal {

class gnss_vertex final : public g2o::BaseVertex<3, Vec3_t> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    gnss_vertex();

    bool read(std::istream& is) override;

    bool write(std::ostream& os) const override;

    void setToOriginImpl() override;

    void oplusImpl(const double* update) override;

	//gnss variance
	//double sigma_sq;
};

inline gnss_vertex::gnss_vertex()
    : g2o::BaseVertex<3, Vec3_t>() {}

inline bool gnss_vertex::read(std::istream& is) {
    Vec3_t lv;
    for (unsigned int i = 0; i < 3; ++i) {
        is >> _estimate(i);
    }
    return true;
}

inline bool gnss_vertex::write(std::ostream& os) const {
    const Vec3_t pos_w = estimate();
    for (unsigned int i = 0; i < 3; ++i) {
        os << pos_w(i) << " ";
    }
    return os.good();
}

inline void gnss_vertex::setToOriginImpl() {
    _estimate.fill(0);
}

inline void gnss_vertex::oplusImpl(const double* update) {
    Eigen::Map<const Vec3_t> v(update);
    _estimate += v;
}

} // namespace internal
} // namespace optimize
} // namespace openvslam

#endif // OPENVSLAM_OPTIMIZER_G2O_GNSS_VERTEX_H


//Added by NFYNT for GNSS vertex node for Hyper-graph optimization




/*
 __  _ _____   ____  _ _____  
|  \| | __\ `v' /  \| |_   _| 
| | ' | _| `. .'| | ' | | |   
|_|\__|_|   !_! |_|\__| |_|
 
*/