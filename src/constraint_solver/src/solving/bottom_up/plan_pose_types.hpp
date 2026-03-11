#ifndef PLAN_POSE_TYPES_HPP
#define PLAN_POSE_TYPES_HPP

// General STD/STL headers
#include <unordered_map>
#include <variant>

// Thirdparty headers
#include <Eigen/Core>

// Custom headers
#include <gcs/model/gcs_data_structures.hpp>

namespace Gcs::Solvers::BottomUp {

struct PointPose {
    Eigen::Vector2d position;
};

struct LinePose {
    Eigen::Vector2d p1;
    Eigen::Vector2d p2;
};

using ElementPose = std::variant<PointPose, LinePose>;
using ClusterPose
    = std::unordered_map<ConstraintGraph::NodeIdType, ElementPose>;

} // namespace Gcs::Solvers::BottomUp

#endif // PLAN_POSE_TYPES_HPP
