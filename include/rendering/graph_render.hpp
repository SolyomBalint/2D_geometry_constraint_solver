#ifndef GRAPH_RENDER_HPP
#define GRAPH_RENDER_HPP

#include <cstdint>
#include <imnodes.h>
#include <string>
#include <vector>

namespace Rendering {

class Edge {
private:
    inline static int32_t m_availableId = 1;
    const int32_t m_id;
    const std::pair<int32_t, int32_t> m_directions;

public:
    explicit Edge(const int32_t input_id, const int32_t output_id)
        : m_id(m_availableId++)
        , m_directions(input_id, output_id)
    {
    }

    void renderEdge();
};

class Node {
private:
    // Inline so that out of class definition can be avoided
    inline static int32_t s_availableAttributeId = 1;
    inline static int32_t s_availableId = 1;
    const static ImU32 s_color = IM_COL32(255, 0, 0, 1);
    const int32_t m_id;
    const int32_t m_inputId;
    const int32_t m_outputId;
    float m_radius;
    ImVec2 m_center;

public:
    explicit Node(float radius, ImVec2 center)
        : m_id(s_availableId++)
        , m_inputId(s_availableAttributeId++)
        , m_outputId(s_availableAttributeId++)
        , m_radius(radius)
        , m_center(center)
    {
    }

    void RenderNode();

    static Edge CreateLink(const Node& inNode, const Node& outNode)
    {
        return Edge(inNode.m_inputId, outNode.m_outputId);
    }
};

class Graph {
private:
    std::vector<Node> m_nodes;
    std::vector<Edge> m_edges;
    constexpr const static std::string s_windowName = "Graph View";

public:
    explicit Graph();

    void RenderGraph();
};
}

#endif // GRAPH_RENDER_HPP
