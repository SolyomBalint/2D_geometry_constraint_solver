#include "graph_render.hpp"

namespace Rendering {

void Edge::renderEdge()
{
    ImNodes::Link(m_id, m_directions.first, m_directions.second);
}

void Node::RenderNode()
{
    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    ImNodes::BeginNode(m_id);
    ImGui::Text("%s", std::to_string(m_id).c_str());

    ImNodes::BeginInputAttribute(m_inputId); // This is wrong
    ImGui::Text("input_id: %s", std::to_string(m_inputId).c_str());
    ImNodes::EndInputAttribute();

    ImNodes::BeginOutputAttribute(m_outputId);
    ImGui::Text("output_id: %s", std::to_string(m_outputId).c_str());
    ImNodes::EndOutputAttribute();

    ImNodes::SetNodeGridSpacePos(m_id, m_center);
    ImNodes::EndNode();
}

Graph::Graph()
{
    m_nodes.push_back(Node { 1.0f, ImVec2 { 200.0f, 200.0f } });
    m_nodes.push_back(Node { 1.0f, ImVec2 { 100.0f, 100.0f } });
    m_edges.push_back(Node::CreateLink(m_nodes[0], m_nodes[1]));
}

void Graph::RenderGraph()
{

    ImGui::Begin(s_windowName.data());
    ImNodes::BeginNodeEditor();

    for (Node& node : m_nodes) {
        node.RenderNode();
    }

    for (Edge& edge : m_edges) {
        edge.renderEdge();
    }

    ImNodes::EndNodeEditor();
    ImGui::End();
}
};
