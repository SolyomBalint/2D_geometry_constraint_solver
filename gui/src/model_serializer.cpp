#include "model_serializer.hpp"

#include "canvas.hpp"

// General STD/STL headers
#include <algorithm>
#include <format>
#include <unordered_map>
#include <utility>

// Thirdparty headers
#include <nlohmann/json.hpp>

namespace Gui {

namespace {

    /// Current file format version.
    constexpr int FORMAT_VERSION = 1;

} // namespace

// ============================================================
// Serialization
// ============================================================

std::expected<std::string, std::string> ModelSerializer::serialize(
    const Canvas& canvas)
{
    try {
        const auto& elements = canvas.getElements();
        const auto& constraints = canvas.getConstraints();

        // Build a sorted list of element IDs so the output order
        // is deterministic and we can map ElementId -> array index.
        std::vector<ElementId> sortedIds;
        sortedIds.reserve(elements.size());
        for (const auto& [id, elem] : elements) {
            sortedIds.push_back(id);
        }
        std::ranges::sort(sortedIds);

        // ElementId -> index in the sorted array.
        std::unordered_map<ElementId, std::size_t> idToIndex;
        for (std::size_t i = 0; i < sortedIds.size(); ++i) {
            idToIndex[sortedIds[i]] = i;
        }

        // Serialize elements in sorted order.
        nlohmann::json elementsJson = nlohmann::json::array();
        for (const auto& eid : sortedIds) {
            const auto& elem = elements.at(eid);
            nlohmann::json elemJson;

            if (elem.type == CanvasElementType::Point) {
                elemJson["type"] = "point";
                elemJson["x"] = elem.x;
                elemJson["y"] = elem.y;
            } else {
                elemJson["type"] = "line";
                elemJson["x1"] = elem.x;
                elemJson["y1"] = elem.y;
                elemJson["x2"] = elem.x2;
                elemJson["y2"] = elem.y2;
            }

            elementsJson.push_back(std::move(elemJson));
        }

        // Serialize constraints in sorted order by ID.
        std::vector<ConstraintId> sortedConstraintIds;
        sortedConstraintIds.reserve(constraints.size());
        for (const auto& [id, constr] : constraints) {
            sortedConstraintIds.push_back(id);
        }
        std::ranges::sort(sortedConstraintIds);

        nlohmann::json constraintsJson = nlohmann::json::array();
        for (const auto& cid : sortedConstraintIds) {
            const auto& constr = constraints.at(cid);
            nlohmann::json constrJson;

            if (constr.type == CanvasConstraintType::Distance) {
                constrJson["type"] = "distance";
            } else {
                constrJson["type"] = "angle";
                constrJson["flipped"] = constr.flipped;
            }

            constrJson["elementA"] = idToIndex.at(constr.elementA);
            constrJson["elementB"] = idToIndex.at(constr.elementB);
            constrJson["value"] = constr.value;

            constraintsJson.push_back(std::move(constrJson));
        }

        // Build the top-level JSON object.
        nlohmann::json root;
        root["version"] = FORMAT_VERSION;
        root["elements"] = std::move(elementsJson);
        root["constraints"] = std::move(constraintsJson);
        root["view"] = {
            { "panX", canvas.getPanX() },
            { "panY", canvas.getPanY() },
            { "zoom", canvas.getZoom() },
        };

        return root.dump(2);

    } catch (const std::exception& e) {
        return std::unexpected(
            std::format("Serialization failed: {}", e.what()));
    }
}

// ============================================================
// Deserialization
// ============================================================

std::expected<ModelData, std::string> ModelSerializer::deserialize(
    const std::string& json)
{
    try {
        auto root = nlohmann::json::parse(json);

        // Version check.
        if (!root.contains("version")) {
            return std::unexpected(std::string("Missing 'version' field"));
        }
        int version = root["version"].get<int>();
        if (version != FORMAT_VERSION) {
            return std::unexpected(
                std::format("Unsupported file version {} (expected {})",
                    version, FORMAT_VERSION));
        }

        ModelData data;

        // Parse elements.
        if (!root.contains("elements") || !root["elements"].is_array()) {
            return std::unexpected(
                std::string("Missing or invalid 'elements' array"));
        }

        for (const auto& elemJson : root["elements"]) {
            ElementData elem;
            std::string type = elemJson.at("type").get<std::string>();

            if (type == "point") {
                elem.type = CanvasElementType::Point;
                elem.x = elemJson.at("x").get<double>();
                elem.y = elemJson.at("y").get<double>();
            } else if (type == "line") {
                elem.type = CanvasElementType::Line;
                elem.x = elemJson.at("x1").get<double>();
                elem.y = elemJson.at("y1").get<double>();
                elem.x2 = elemJson.at("x2").get<double>();
                elem.y2 = elemJson.at("y2").get<double>();
            } else {
                return std::unexpected(
                    std::format("Unknown element type: '{}'", type));
            }

            data.elements.push_back(elem);
        }

        // Parse constraints.
        if (root.contains("constraints") && root["constraints"].is_array()) {
            for (const auto& constrJson : root["constraints"]) {
                ConstraintData constr;
                std::string type = constrJson.at("type").get<std::string>();

                if (type == "distance") {
                    constr.type = CanvasConstraintType::Distance;
                } else if (type == "angle") {
                    constr.type = CanvasConstraintType::Angle;
                    constr.flipped = constrJson.value("flipped", false);
                } else {
                    return std::unexpected(
                        std::format("Unknown constraint type: '{}'", type));
                }

                constr.elementA = constrJson.at("elementA").get<std::size_t>();
                constr.elementB = constrJson.at("elementB").get<std::size_t>();
                constr.value = constrJson.at("value").get<double>();

                // Validate element indices.
                if (constr.elementA >= data.elements.size()
                    || constr.elementB >= data.elements.size()) {
                    return std::unexpected(
                        std::format("Constraint references invalid element "
                                    "index ({} or {}; {} elements exist)",
                            constr.elementA, constr.elementB,
                            data.elements.size()));
                }

                data.constraints.push_back(constr);
            }
        }

        // Parse view state (optional).
        if (root.contains("view") && root["view"].is_object()) {
            const auto& view = root["view"];
            data.panX = view.value("panX", 0.0);
            data.panY = view.value("panY", 0.0);
            data.zoom = view.value("zoom", 1.0);
        }

        return data;

    } catch (const nlohmann::json::exception& e) {
        return std::unexpected(std::format("JSON parse error: {}", e.what()));
    } catch (const std::exception& e) {
        return std::unexpected(
            std::format("Deserialization failed: {}", e.what()));
    }
}

} // namespace Gui
