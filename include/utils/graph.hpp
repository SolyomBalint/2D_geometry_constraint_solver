#ifndef GRAPH_HPP
#define GRAPH_HPP

#include <memory>
#include <vector>
#include <uuid/uuid.h>

#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>

namespace MathUtils {

const auto graphLogger = spdlog::stdout_color_mt("GRAPH");

class Node;

struct Edge {
    std::shared_ptr<Node> m_neighbour;
    uuid_t uuid;

    explicit Edge(std::shared_ptr<Node> neighbour)
        : m_neighbour(neighbour)
    {
        uuid_generate(uuid);
    }

    Edge(const Edge&) = default;
    Edge(Edge&&) = default;
    Edge& operator=(const Edge&) = default;
    Edge& operator=(Edge&&) = default;

    bool operator==(const Edge& other) const
    {
        // Compare memory addresses to check if they are the same object
        return this == &other;
    }
};

// TODO think this trough because of directed graphs
struct Node {
    std::vector<Edge> m_edges;
    uuid_t uuid;

    explicit Node()
        : m_edges({})
    {
        uuid_generate(uuid);
    }
    Node(const Node&) = default;
    Node(Node&&) = default;
    Node& operator=(const Node&) = default;
    Node& operator=(Node&&) = default;

    bool operator==(const Node& other) const
    {
        // Compare memory addresses to check if they are the same object
        return this == &other;
    }
};

class Graph {
public:
    virtual std::vector<Node> getCutVertices() = 0;
};

class UndirectedGraph : public Graph {
public:
    std::vector<Node> getCutVertices();

private:
    std::vector<Node> m_adjacencyList;
};

}

namespace std {

template <>
struct hash<MathUtils::Edge> {
    std::size_t operator()(const MathUtils::Edge& edge) const noexcept
    {
        // Use the UUID of the edge for hashing
        return std::hash<std::string_view>()(std::string_view(reinterpret_cast<const char*>(edge.uuid), sizeof(uuid_t)));
    }
};

template <>
struct hash<MathUtils::Node> {
    std::size_t operator()(const MathUtils::Node& node) const noexcept
    {
        // Use the UUID of the node for hashing
        return std::hash<std::string_view>()(std::string_view(reinterpret_cast<const char*>(node.uuid), sizeof(uuid_t)));
    }
};

} // namespace std
#endif // GRAPH_HPP
