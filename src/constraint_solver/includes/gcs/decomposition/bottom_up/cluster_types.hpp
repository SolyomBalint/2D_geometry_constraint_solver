#ifndef GCS_DECOMPOSITION_BOTTOM_UP_CLUSTER_TYPES_HPP
#define GCS_DECOMPOSITION_BOTTOM_UP_CLUSTER_TYPES_HPP

// General STD/STL headers
#include <algorithm>
#include <cassert>
#include <compare>
#include <concepts>
#include <cstddef>
#include <functional>
#include <iterator>
#include <ranges>
#include <span>
#include <utility>
#include <vector>

// Custom headers
#include <gcs/model/gcs_data_structures.hpp>

namespace Gcs {

// ============================================================
// ClusterId
// ============================================================

/// @brief Strongly-typed identifier for reduction clusters.
struct ClusterId {
    int value {};

    bool operator==(const ClusterId&) const = default;
    auto operator<=>(const ClusterId&) const = default;
};

// ============================================================
// Cluster
// ============================================================

/// @brief Semantic cluster representation used by bottom-up reduction.
struct Cluster {
    using ElementIdType = ConstraintGraph::NodeIdType;

    ClusterId id;
    std::vector<ElementIdType> elements;

    /// @brief Number of geometric elements in this cluster.
    [[nodiscard]] std::size_t elementCount() const { return elements.size(); }
};

// ============================================================
// Helpers
// ============================================================

/**
 * @brief Canonicalize a list of cluster elements.
 *
 * Produces a sorted, unique element list.
 */
[[nodiscard]] inline std::vector<ConstraintGraph::NodeIdType>
makeCanonicalClusterElements(
    std::span<const ConstraintGraph::NodeIdType> elements)
{
    std::vector<ConstraintGraph::NodeIdType> canonical { elements.begin(),
        elements.end() };
    std::ranges::sort(canonical);
    auto uniqueBegin = std::ranges::unique(canonical).begin();
    canonical.erase(uniqueBegin, canonical.end());
    return canonical;
}

/**
 * @brief Build a canonical cluster from arbitrary element ordering.
 * @pre @p elements must not be empty.
 */
[[nodiscard]] inline Cluster makeCluster(
    ClusterId id, std::span<const ConstraintGraph::NodeIdType> elements)
{
    auto canonicalElements = makeCanonicalClusterElements(elements);
    assert(!canonicalElements.empty()
        && "makeCluster: elements must not be empty");
    return Cluster { .id = id, .elements = std::move(canonicalElements) };
}

/**
 * @brief Union of two canonical element sets.
 *
 * Both input sets are expected to be sorted and unique.
 * Output is also sorted and unique.
 */
[[nodiscard]] inline std::vector<ConstraintGraph::NodeIdType>
clusterElementsUnion(std::span<const ConstraintGraph::NodeIdType> lhs,
    std::span<const ConstraintGraph::NodeIdType> rhs)
{
    std::vector<ConstraintGraph::NodeIdType> unionElements;
    unionElements.reserve(lhs.size() + rhs.size());
    std::ranges::set_union(lhs, rhs, std::back_inserter(unionElements));
    return unionElements;
}

/**
 * @brief Union of three canonical element sets.
 *
 * All input sets are expected to be sorted and unique.
 * Output is also sorted and unique.
 */
[[nodiscard]] inline std::vector<ConstraintGraph::NodeIdType>
clusterElementsUnion(std::span<const ConstraintGraph::NodeIdType> a,
    std::span<const ConstraintGraph::NodeIdType> b,
    std::span<const ConstraintGraph::NodeIdType> c)
{
    auto firstUnion = clusterElementsUnion(a, b);
    return clusterElementsUnion(firstUnion, c);
}

/**
 * @brief Compare two canonical element sets for equality.
 */
[[nodiscard]] inline bool areSameClusterElements(
    std::span<const ConstraintGraph::NodeIdType> lhs,
    std::span<const ConstraintGraph::NodeIdType> rhs)
{
    return std::ranges::equal(lhs, rhs);
}

static_assert(std::totally_ordered<ClusterId>);

} // namespace Gcs

template <>
struct std::hash<Gcs::ClusterId> {
    std::size_t operator()(const Gcs::ClusterId& id) const noexcept
    {
        return std::hash<int> {}(id.value);
    }
};

#endif // GCS_DECOMPOSITION_BOTTOM_UP_CLUSTER_TYPES_HPP
