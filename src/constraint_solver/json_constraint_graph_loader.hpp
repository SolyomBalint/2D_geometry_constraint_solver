#ifndef JSON_CONSTRAINT_GRAPH_LOADER_HPP
#define JSON_CONSTRAINT_GRAPH_LOADER_HPP

#include "geometric_constraint_system.hpp"
#include <string>
#include <optional>

namespace Gcs {

/**
 * @brief Load a ConstraintGraph from a JSON file exported by the Python GUI
 *
 * The JSON file format is as follows:
 * {
 *   "version": "1.0",
 *   "shapes": [
 *     {
 *       "type": "Point",
 *       "x": 100.0,
 *       "y": 200.0,
 *       "line_width": 5.0,
 *       "colour": {"r": 0.0, "g": 0.0, "b": 0.0},
 *       "radius": 5.0,
 *       "visible": true
 *     },
 *     {
 *       "type": "Circle",
 *       "center_x": 300.0,
 *       "center_y": 400.0,
 *       "radius": 50.0,
 *       "line_width": 5.0,
 *       "colour": {"r": 0.0, "g": 0.0, "b": 1.0},
 *       "visible": true
 *     },
 *     {
 *       "type": "Line",
 *       "point_one": {...},
 *       "point_two": {...},
 *       "line_width": 5.0,
 *       "colour": {"r": 0.0, "g": 0.0, "b": 0.0},
 *       "visible": true
 *     }
 *   ],
 *   "constraints": [
 *     {
 *       "shape1_index": 0,
 *       "shape2_index": 1,
 *       "constraint_type": "distance",
 *       "value": 100.0
 *     }
 *   ]
 * }
 *
 * @param filename Path to the JSON file
 * @return ConstraintGraph loaded from the file, or std::nullopt if loading failed
 */
std::optional<ConstraintGraph> loadConstraintGraphFromJson(const std::string& filename);

} // namespace Gcs

#endif // JSON_CONSTRAINT_GRAPH_LOADER_HPP
