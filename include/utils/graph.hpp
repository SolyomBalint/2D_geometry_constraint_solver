#ifndef GRAPH_HPP
#define GRAPH_HPP

#include <vector>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>

namespace MathUtils {

/*static const auto graphLogger = spdlog::stdout_color_mt("GRAPH");*/

class Node;

struct Edge {
    const Node& m_neighbour;

    explicit Edge(const Node& neighbour)
        : m_neighbour(neighbour)
    {
    }

    Edge(const Edge&) = default;
    Edge(Edge&&) = default;

    bool operator==(const Edge& other) const
    {
        // Compare memory addresses to check if they are the same object
        return this == &other;
    }
};

// TODO think this trough because of directed graphs
struct Node {
    std::string tempName = std::format("node{}", tempCoutner++); ///< Temporary naming logic, helps in printf debugging

    std::vector<Edge> m_edges;

    explicit Node()
        : m_edges({})
        , m_uuid(boost::uuids::random_generator()())
    {
    }
    Node(const Node&) = default;
    Node(Node&&) = default;
    Node& operator=(const Node&) = default;
    Node& operator=(Node&&) = default;

    bool operator==(const Node& other) const
    {
        return m_uuid == other.m_uuid;
    }

    /**
     * Returns hash value based on uuid
    */
    std::size_t operator()(MathUtils::Node const& node) const noexcept
    {
        return std::hash<boost::uuids::uuid> {}(node.m_uuid);
    }

private:
    inline static int tempCoutner = 1;
    boost::uuids::uuid m_uuid;
};

class Graph {
public:
    virtual std::vector<Node> getCutVertices() = 0;
};

class UndirectedGraph : public Graph {
public:
    std::vector<Node> getCutVertices();

    explicit UndirectedGraph(std::vector<Node> inList)
        : Graph()
        , m_adjacencyList(inList)
    {
    }

private:
    std::vector<Node> m_adjacencyList;
};

}

#endif // GRAPH_HPP
