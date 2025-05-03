#ifndef GRAPH_HPP
#define GRAPH_HPP

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <cstddef>
#include <format>
#include <functional>
#include <string>
#include <utility>
#include <vector>

namespace MathUtils {

struct Node;

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
        , m_uuid_(boost::uuids::random_generator()())
    {
    }

    Node(const Node&) = default;
    Node(Node&&) = default;
    Node& operator=(const Node&) = default;
    Node& operator=(Node&&) = default;

    bool operator==(const Node& other) const { return m_uuid_ == other.m_uuid_; }

    /**
     * Returns hash value based on uuid
     */
    std::size_t operator()(MathUtils::Node const& node) const noexcept
    {
        return std::hash<boost::uuids::uuid> {}(node.m_uuid_);
    }

private:
    inline static int tempCoutner = 1;
    boost::uuids::uuid m_uuid_;
};

class Graph {
public:
    Graph(Graph&) = default;
    Graph(Graph&&) = default;
    Graph& operator=(const Graph&) = default;
    Graph& operator=(const Graph&&) = delete;
    virtual ~Graph() = default;
    virtual std::vector<Node> getCutVertices() = 0;
};

class UndirectedGraph : public Graph {
public:
    std::vector<Node> getCutVertices() override;

    explicit UndirectedGraph(std::vector<Node> inList)
        : Graph()
        , m_adjacencyList_(std::move(inList))
    {
    }

    const std::vector<Node>& getAdjList() const { return m_adjacencyList_; }

private:
    std::vector<Node> m_adjacencyList_;
};

} // namespace MathUtils

#endif // GRAPH_HPP
