#ifndef GRAPH_HPP
#define GRAPH_HPP

#include <common/common_uuid.hpp>
#include <format>
#include <string>
#include <utility>
#include <uuid.h>
#include <vector>

namespace MathUtils {

class Node;

class Edge {
public:
    const Node& m_neighbour;

    explicit Edge(const Node& neighbour)
        : m_neighbour(neighbour)
    {
    }

    Edge(const Edge&) = default;
    Edge(Edge&&) = default;
    Edge& operator=(const Edge&) = delete;
    Edge& operator=(Edge&&) = delete;
    ~Edge() = default;

    bool operator==(const Edge& other) const
    {
        // Compare memory addresses to check if they are the same object
        return this == &other;
    }
};

// TODO think this trough because of directed graphs
class Node {
public:
    std::string tempName = std::format("node{}", tempCoutner++); ///< Temporary naming logic, helps in printf debugging

    // TODO: fix this
    std::vector<Edge> m_edges;

    explicit Node()
        : m_edges({})
        , m_uuid_(common::uuid::generateUuidMt19937())
    {
    }

    Node(const Node&) = default;
    Node(Node&&) = default;
    Node& operator=(const Node&) = default;
    Node& operator=(Node&&) = default;
    ~Node() = default;

    bool operator==(const Node& other) const { return m_uuid_ == other.m_uuid_; }

    const uuids::uuid& getUuId() const { return m_uuid_; }

private:
    inline static int tempCoutner = 1;
    uuids::uuid m_uuid_;
};

class Graph {
public:
    Graph() = default;
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
        : m_adjacencyList_(std::move(inList))
    {
    }

    const std::vector<Node>& getAdjList() const { return m_adjacencyList_; }

private:
    std::vector<Node> m_adjacencyList_;
};

} // namespace MathUtils

#endif // GRAPH_HPP
