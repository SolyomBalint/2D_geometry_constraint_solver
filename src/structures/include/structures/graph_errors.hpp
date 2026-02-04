#ifndef GRAPH_ERRORS_HPP
#define GRAPH_ERRORS_HPP

namespace MathUtils {

/// @brief Errors from graph mutation operations (concept-level).
///
/// Every GraphTopology implementation uses this enum for addEdge,
/// removeNode, and removeEdge return values.
enum class GraphError {
    NodeNotFound, ///< A referenced node does not exist in the graph.
    EdgeNotFound, ///< A referenced edge does not exist in the graph.
    InternalError, ///< Implementation-specific invariant violation.
};

/// @brief Errors from SubGraph ID-mapping lookups.
///
/// Used by SubGraph::originalNodeId, localNodeId, originalEdgeId,
/// and localEdgeId.
enum class SubGraphMappingError {
    NodeMappingNotFound, ///< No mapping exists for the given node ID.
    EdgeMappingNotFound, ///< No mapping exists for the given edge ID.
};

/// @brief Errors from PropertyMap access operations.
///
/// Used by PropertyMap::get and PropertyMap::erase.
enum class PropertyMapError {
    KeyNotFound, ///< The requested key does not exist in the property map.
};

} // namespace MathUtils

#endif // GRAPH_ERRORS_HPP
