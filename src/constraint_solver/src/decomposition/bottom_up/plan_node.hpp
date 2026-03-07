#ifndef PLAN_NODE_HPP
#define PLAN_NODE_HPP

// General STD/STL headers
#include <array>
#include <variant>
#include <vector>

// Custom headers
#include "cluster_types.hpp"

namespace Gcs {

enum class PlanNodeKind {
    TrianglePrimitive,
    EdgePrimitive,
    Merge3,
    Merge2,
};

struct TrianglePrimitiveInfo {
    ClusterId cluster;
    std::array<ConstraintGraph::NodeIdType, 3> elements;
};

struct EdgePrimitiveInfo {
    ClusterId cluster;
    std::array<ConstraintGraph::NodeIdType, 2> elements;
};

struct Merge3Info {
    ClusterId output;
    std::array<ClusterId, 3> inputs;
    std::vector<ConstraintGraph::NodeIdType> outputElements;
};

struct Merge2Info {
    ClusterId output;
    std::array<ClusterId, 2> inputs;
    std::vector<ConstraintGraph::NodeIdType> outputElements;
};

struct PlanNode {
    PlanNodeKind kind;
    std::variant<EdgePrimitiveInfo, TrianglePrimitiveInfo, Merge3Info,
        Merge2Info>
        info;
};

} // namespace Gcs

#endif // PLAN_NODE_HPP
