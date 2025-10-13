#include "json_constraint_graph_loader.hpp"
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <unordered_map>
#include <spdlog/spdlog.h>

namespace Gcs {

namespace {
    // Helper to flip Y coordinate from canvas space to solver space
    inline double flipYForSolver(double canvasY) { return -canvasY; }

    // Helper to create a Point element from JSON
    std::shared_ptr<Element> createPointFromJson(const nlohmann::json& shapeData)
    {
        double canvasX = shapeData.at("x").get<double>();
        double canvasY = shapeData.at("y").get<double>();

        // Convert canvas coordinates to solver coordinates (flip Y)
        Point point(canvasX, flipYForSolver(canvasY));

        return std::make_shared<Element>(point);
    }

    // Helper to create a FixedRadiusCircle element from JSON
    std::shared_ptr<Element> createCircleFromJson(const nlohmann::json& shapeData)
    {
        double canvasX = shapeData.at("center_x").get<double>();
        double canvasY = shapeData.at("center_y").get<double>();
        double radius = shapeData.at("radius").get<double>();

        // Convert canvas coordinates to solver coordinates (flip Y)
        FixedRadiusCircle circle(canvasX, flipYForSolver(canvasY), radius);

        return std::make_shared<Element>(circle);
    }

    // Helper to create a Line element from JSON
    std::shared_ptr<Element> createLineFromJson(const nlohmann::json& shapeData)
    {
        const auto& point1Data = shapeData.at("point_one");
        const auto& point2Data = shapeData.at("point_two");

        double canvasX1 = point1Data.at("x").get<double>();
        double canvasY1 = point1Data.at("y").get<double>();
        double canvasX2 = point2Data.at("x").get<double>();
        double canvasY2 = point2Data.at("y").get<double>();

        // Convert canvas coordinates to solver coordinates (flip Y)
        Line line(canvasX1, flipYForSolver(canvasY1),
                  canvasX2, flipYForSolver(canvasY2));

        return std::make_shared<Element>(line);
    }

    // Helper to create a Constraint from JSON
    std::shared_ptr<Constraint> createConstraintFromJson(const nlohmann::json& constraintData)
    {
        std::string constraintType = constraintData.at("constraint_type").get<std::string>();

        // Convert to lowercase for case-insensitive comparison
        std::transform(constraintType.begin(), constraintType.end(),
                      constraintType.begin(), ::tolower);

        if (constraintType == "distance") {
            double value = constraintData.at("value").get<double>();
            return std::make_shared<Constraint>(DistanceConstraint(value));
        }
        else if (constraintType == "tangency") {
            double value = constraintData.at("value").get<double>();
            return std::make_shared<Constraint>(TangencyConstraint(value));
        }
        else if (constraintType == "pointonline") {
            return std::make_shared<Constraint>(PointOnLineConstraint());
        }
        else {
            spdlog::warn("Unknown constraint type: {}", constraintType);
            return nullptr;
        }
    }
}

std::optional<ConstraintGraph> loadConstraintGraphFromJson(const std::string& filename)
{
    try {
        // Read JSON file
        std::ifstream file(filename);
        if (!file.is_open()) {
            spdlog::error("Failed to open file: {}", filename);
            return std::nullopt;
        }

        nlohmann::json jsonData;
        file >> jsonData;
        file.close();

        // Validate version
        if (jsonData.contains("version")) {
            std::string version = jsonData.at("version").get<std::string>();
            if (version != "1.0") {
                spdlog::warn("Unsupported JSON version: {}", version);
            }
        }

        // Create an empty ConstraintGraph
        ConstraintGraph graph;

        // Parse shapes and add as nodes
        std::vector<ConstraintGraph::NodeType> nodeRefs;
        std::unordered_map<std::size_t, std::size_t> shapeIndexToNodeIndex;

        if (!jsonData.contains("shapes")) {
            spdlog::error("JSON file missing 'shapes' field");
            return std::nullopt;
        }

        const auto& shapes = jsonData.at("shapes");
        for (std::size_t i = 0; i < shapes.size(); ++i) {
            const auto& shapeData = shapes[i];

            // Skip shapes that are not visible (not geometric elements)
            if (shapeData.contains("visible") && !shapeData.at("visible").get<bool>()) {
                spdlog::debug("Skipped invisible shape at index {} (not a geometric element)", i);
                continue;
            }

            std::string shapeType = shapeData.at("type").get<std::string>();
            std::shared_ptr<Element> element;

            if (shapeType == "Point") {
                element = createPointFromJson(shapeData);
            }
            else if (shapeType == "Circle") {
                element = createCircleFromJson(shapeData);
            }
            else if (shapeType == "Line") {
                element = createLineFromJson(shapeData);
            }
            else {
                spdlog::warn("Unknown shape type: {}", shapeType);
                continue;
            }

            if (element) {
                auto node = graph.addNode(element);
                shapeIndexToNodeIndex[i] = nodeRefs.size();
                nodeRefs.push_back(node);
                spdlog::debug("Added {} to graph as node {} (shape index {})",
                             shapeType, nodeRefs.size() - 1, i);
            }
        }

        // Parse constraints and add as edges
        if (!jsonData.contains("constraints")) {
            spdlog::warn("JSON file missing 'constraints' field, graph will have no edges");
        } else {
            const auto& constraints = jsonData.at("constraints");
            for (const auto& constraintData : constraints) {
                std::size_t shape1Idx = constraintData.at("shape1_index").get<std::size_t>();
                std::size_t shape2Idx = constraintData.at("shape2_index").get<std::size_t>();

                // Skip constraints referencing invisible shapes
                if (!shapeIndexToNodeIndex.count(shape1Idx) || !shapeIndexToNodeIndex.count(shape2Idx)) {
                    spdlog::debug("Skipped constraint referencing invisible shape(s) at indices {} and {}",
                                 shape1Idx, shape2Idx);
                    continue;
                }

                if (shape1Idx >= shapes.size() || shape2Idx >= shapes.size()) {
                    spdlog::error("Invalid shape indices: {} and {}", shape1Idx, shape2Idx);
                    continue;
                }

                auto constraint = createConstraintFromJson(constraintData);
                if (constraint) {
                    std::size_t node1Idx = shapeIndexToNodeIndex[shape1Idx];
                    std::size_t node2Idx = shapeIndexToNodeIndex[shape2Idx];
                    graph.addEdge(nodeRefs[node1Idx], nodeRefs[node2Idx], constraint);
                    spdlog::debug("Added {} constraint between nodes {} and {} (shape indices {} and {})",
                                 constraint->getConstraintName(), node1Idx, node2Idx, shape1Idx, shape2Idx);
                }
            }
        }

        spdlog::info("Successfully loaded constraint graph from {}", filename);
        spdlog::info("  Nodes: {}, Edges: {}", graph.getNodeCount(), graph.getEdgeCount());

        return graph;

    } catch (const nlohmann::json::exception& e) {
        spdlog::error("JSON parsing error: {}", e.what());
        return std::nullopt;
    } catch (const std::exception& e) {
        spdlog::error("Error loading constraint graph: {}", e.what());
        return std::nullopt;
    }
}

} // namespace Gcs
