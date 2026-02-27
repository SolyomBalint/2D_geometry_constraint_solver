#ifndef GUI_MODEL_SERIALIZER_HPP
#define GUI_MODEL_SERIALIZER_HPP

// General STD/STL headers
#include <expected>
#include <string>
#include <vector>

// Custom headers
#include "./canvas_types.hpp"

namespace Gui {

class Canvas;

/**
 * @brief Serialization data for a single geometric element.
 *
 * Holds the type and position data extracted from a CanvasElement.
 * Used as an intermediate representation during save/load.
 */
struct ElementData {
    CanvasElementType type;
    double x = 0.0;
    double y = 0.0;
    double x2 = 0.0; ///< Second endpoint (lines only).
    double y2 = 0.0; ///< Second endpoint (lines only).
};

/**
 * @brief Serialization data for a single geometric constraint.
 *
 * Holds the type, value, connected element indices, and
 * orientation extracted from a CanvasConstraint. Element indices
 * refer to positions in the accompanying ElementData vector.
 */
struct ConstraintData {
    CanvasConstraintType type;
    std::size_t elementA = 0; ///< Index into the elements array.
    std::size_t elementB = 0; ///< Index into the elements array.
    double value = 0.0;
    bool flipped = false; ///< Angle orientation flag.
};

/**
 * @brief Complete serialized model state.
 *
 * Contains all elements, constraints, and viewport state needed
 * to fully reconstruct a model.
 */
struct ModelData {
    std::vector<ElementData> elements;
    std::vector<ConstraintData> constraints;
    double panX = 0.0;
    double panY = 0.0;
    double zoom = 1.0;
};

/**
 * @brief Handles serialization and deserialization of the modeller
 *        state to/from JSON.
 *
 * The serializer works with the Canvas's flat visual state
 * (CanvasElement / CanvasConstraint maps) rather than the
 * constraint graph directly. On deserialization it produces a
 * ModelData struct; the caller replays creation commands through
 * the ConstraintModel API to rebuild the constraint graph.
 */
class ModelSerializer {
public:
    /**
     * @brief Serialize the current canvas state to a JSON string.
     * @param canvas The canvas whose elements, constraints, and
     *        viewport state should be serialized.
     * @return JSON string on success, or an error message.
     */
    [[nodiscard]] static std::expected<std::string, std::string> serialize(
        const Canvas& canvas);

    /**
     * @brief Deserialize a JSON string into a ModelData struct.
     * @param json The JSON string to parse.
     * @return Parsed ModelData on success, or an error message.
     */
    [[nodiscard]] static std::expected<ModelData, std::string> deserialize(
        const std::string& json);
};

} // namespace Gui

#endif // GUI_MODEL_SERIALIZER_HPP
