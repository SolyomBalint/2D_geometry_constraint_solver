import os
import sys
import json
import math

# Add path to C++ bindings
sys.path.append(os.path.join(os.path.dirname(__file__), '../../../../build/Debug'))

BINDINGS_AVAILABLE = False
gcs_handle = None

try:
    import constraint_graph_handle_binding as gcs_handle
    BINDINGS_AVAILABLE = True
    print("✓ Successfully imported constraint_graph_handle_binding")
except ImportError as e:
    print(f"✗ Failed to import constraint_graph_handle_binding: {e}")
    print("  GUI will operate in limited mode without solver")
    BINDINGS_AVAILABLE = False

from . import drawmodel


class GeometricConstraintSystem:
    """Wrapper for the C++ ConstraintGraphHandle with Python-friendly interface"""

    def __init__(self, shape_data):
        self.shape_data_ref = shape_data

        # Create a C++ ConstraintGraphHandle if bindings available
        if BINDINGS_AVAILABLE and gcs_handle is not None:
            self.graph_handle = gcs_handle.ConstraintGraphHandle()
        else:
            self.graph_handle = None

        # Map drawable shapes to constraint graph node IDs
        self.shape_to_node_id = {}

        # Track constraints between shapes
        self.constraints = []  # List of (shape1, shape2, constraint_type, value) tuples

    def add_shape_as_node(self, shape: drawmodel.Drawable):
        """Add a drawable shape as a node in the constraint graph"""
        if not BINDINGS_AVAILABLE or self.graph_handle is None:
            print("✗ Cannot add shape: C++ bindings not available")
            return None

        try:
            # Convert drawable shape to graph node using handle API
            if isinstance(shape, drawmodel.Point):
                node_id = self.graph_handle.addPoint(
                    shape.coords.x, shape.coords.y
                )
            elif isinstance(shape, drawmodel.Circle):
                node_id = self.graph_handle.addCircle(
                    shape.center.x, shape.center.y, shape.radius
                )
            elif isinstance(shape, drawmodel.Line):
                # For lines, we use two-point representation
                x1 = shape.defining_point_one.coords.x
                y1 = shape.defining_point_one.coords.y
                x2 = shape.defining_point_two.coords.x
                y2 = shape.defining_point_two.coords.y
                node_id = self.graph_handle.addLine(x1, y1, x2, y2)
            else:
                print(f"Warning: Unknown shape type {type(shape)}")
                return None

            self.shape_to_node_id[shape] = node_id
            print(f"Added shape {type(shape).__name__} as node {node_id}")
            return node_id

        except Exception as e:
            print(f"Error adding shape as node: {e}")
            import traceback
            traceback.print_exc()
            return None

    def add_constraint_between_shapes(
        self,
        shape1: drawmodel.Drawable,
        shape2: drawmodel.Drawable,
        constraint_type: str,
        value: float,
    ):
        """Add a constraint between two shapes"""
        if not BINDINGS_AVAILABLE or self.graph_handle is None:
            print("✗ Cannot add constraint: C++ bindings not available")
            return False

        # Ensure both shapes are in the constraint graph
        node1_id = self.shape_to_node_id.get(shape1)
        node2_id = self.shape_to_node_id.get(shape2)

        if node1_id is None:
            node1_id = self.add_shape_as_node(shape1)
        if node2_id is None:
            node2_id = self.add_shape_as_node(shape2)

        if node1_id is None or node2_id is None:
            print("Error: Could not add shapes as nodes")
            return False

        try:
            # Add constraint using handle API
            success = False
            if constraint_type.lower() == "distance":
                success = self.graph_handle.addDistanceConstraint(
                    node1_id, node2_id, value
                )
            elif constraint_type.lower() == "tangency":
                success = self.graph_handle.addTangencyConstraint(
                    node1_id, node2_id, value
                )
            elif constraint_type.lower() == "pointonline":
                success = self.graph_handle.addPointOnLineConstraint(
                    node1_id, node2_id
                )
            else:
                print(f"Unknown constraint type: {constraint_type}")
                return False

            if success:
                # Track the constraint
                self.constraints.append((shape1, shape2, constraint_type, value))
                print(
                    f"Added {constraint_type} constraint (value: {value}) between nodes {node1_id} and {node2_id}"
                )

            return success

        except Exception as e:
            print(f"Error adding constraint: {e}")
            import traceback
            traceback.print_exc()
            return False

    def get_constraint_graph_info(self, include_decomposition=False):
        """Get information about the current constraint graph

        Args:
            include_decomposition: If True, include expensive decomposition analysis
        """
        if not BINDINGS_AVAILABLE or self.graph_handle is None:
            return {
                "nodes": 0,
                "edges": 0,
                "well_constrained": False,
                "subgraph_sizes": [],
            }

        try:
            node_count = self.graph_handle.getNodeCount()
            edge_count = self.graph_handle.getEdgeCount()
            well_constrained = self.graph_handle.isWellConstrained()

            # Only compute decomposition if explicitly requested (it's expensive)
            subgraph_sizes = []
            if include_decomposition:
                subgraph_sizes = self.graph_handle.getDecompositionInfo()

            return {
                "nodes": node_count,
                "edges": edge_count,
                "well_constrained": well_constrained,
                "subgraph_sizes": subgraph_sizes if subgraph_sizes else [node_count] if node_count > 0 else [],
            }
        except Exception as e:
            print(f"Error getting constraint graph info: {e}")
            import traceback
            traceback.print_exc()
            return {
                "nodes": 0,
                "edges": 0,
                "well_constrained": False,
                "subgraph_sizes": [],
            }

    def solve_constraint_graph(self):
        """Solve the constraint graph and update shape positions"""
        if not BINDINGS_AVAILABLE or self.graph_handle is None:
            print("✗ C++ bindings not available for solving")
            return False

        try:
            node_count = self.graph_handle.getNodeCount()
            if node_count == 0:
                print("✗ No nodes in graph to solve")
                return False

            print(f"Solving constraint system with {node_count} nodes and {self.graph_handle.getEdgeCount()} edges")

            # Create reverse mapping from node_id to shape
            node_id_to_shape = {node_id: shape for shape, node_id in self.shape_to_node_id.items()}

            # Call the solver
            solved_positions = self.graph_handle.solve()

            print(f"✓ Solver returned {len(solved_positions)} positions")

            if len(solved_positions) == 0:
                print("✗ No positions returned from solver")
                return False

            # Create reverse mapping from node ID to shape (already created above)
            # node_id_to_shape = {node_id: shape for shape, node_id in self.shape_to_node_id.items()}

            # Calculate centroids of original and solved positions
            original_centroid_x = 0.0
            original_centroid_y = 0.0
            solved_centroid_x = 0.0
            solved_centroid_y = 0.0
            point_count = 0

            for node_pos in solved_positions:
                node_id = node_pos.nodeId
                if node_id not in node_id_to_shape:
                    continue

                shape = node_id_to_shape[node_id]
                data = node_pos.data

                # Get original position
                if isinstance(shape, drawmodel.Point):
                    original_centroid_x += shape.coords.x
                    original_centroid_y += shape.coords.y
                    if len(data) >= 2:
                        solved_centroid_x += data[0]
                        solved_centroid_y += data[1]
                    point_count += 1

                elif isinstance(shape, drawmodel.Circle):
                    original_centroid_x += shape.center.x
                    original_centroid_y += shape.center.y
                    if len(data) >= 2:
                        solved_centroid_x += data[0]
                        solved_centroid_y += data[1]
                    point_count += 1

                elif isinstance(shape, drawmodel.Line):
                    # Use line reference point for centroid calculation
                    original_centroid_x += shape.defining_point_one.coords.x
                    original_centroid_y += shape.defining_point_one.coords.y
                    if len(data) >= 2:
                        solved_centroid_x += data[0]
                        solved_centroid_y += data[1]
                    point_count += 1

            if point_count > 0:
                original_centroid_x /= point_count
                original_centroid_y /= point_count
                solved_centroid_x /= point_count
                solved_centroid_y /= point_count

                # Calculate translation to align solved centroid with original centroid
                translate_x = original_centroid_x - solved_centroid_x
                translate_y = original_centroid_y - solved_centroid_y

                print(f"  Original centroid: ({original_centroid_x:.2f}, {original_centroid_y:.2f})")
                print(f"  Solved centroid: ({solved_centroid_x:.2f}, {solved_centroid_y:.2f})")
                print(f"  Translating coordinates by ({translate_x:.2f}, {translate_y:.2f}) to maintain original position")

                # Apply translation to all solved positions
                translated_positions = []
                for node_pos in solved_positions:
                    data = node_pos.data
                    translated_data = data.copy()
                    if len(translated_data) >= 2:
                        translated_data[0] += translate_x
                        translated_data[1] += translate_y
                    translated_positions.append((node_pos.nodeId, translated_data))

                # Replace solved_positions with translated version
                solved_positions_dict = {node_id: data for node_id, data in translated_positions}
            else:
                # No translation needed, use original positions
                solved_positions_dict = {node_pos.nodeId: node_pos.data for node_pos in solved_positions}

            # Update shapes with solved (and translated) positions
            for node_id, data in solved_positions_dict.items():
                if node_id not in node_id_to_shape:
                    print(f"⚠ Warning: Node {node_id} has no shape mapping")
                    continue

                shape = node_id_to_shape[node_id]

                # Update shape based on type
                # Note: ConstraintGraphHandle already returns coordinates in canvas space
                if isinstance(shape, drawmodel.Point):
                    if len(data) >= 2:
                        shape.coords.x = data[0]
                        shape.coords.y = data[1]
                        print(f"  Updated Point (node {node_id}) to ({data[0]:.2f}, {data[1]:.2f})")

                elif isinstance(shape, drawmodel.Circle):
                    if len(data) >= 3:
                        shape.center.x = data[0]
                        shape.center.y = data[1]
                        shape.radius = data[2]
                        print(f"  Updated Circle (node {node_id}) center to ({data[0]:.2f}, {data[1]:.2f}), radius {data[2]:.2f}")

                elif isinstance(shape, drawmodel.Line):
                    # Lines will be updated separately after all points are positioned
                    print(f"  Skipping Line (node {node_id}) - will update after points")

            # After all points are updated, update lines based on their constrained points
            print("Updating lines based on constrained points...")
            for node_id, shape in node_id_to_shape.items():
                if isinstance(shape, drawmodel.Line):
                    # Find all points constrained to this line via PointOnLine constraints
                    constrained_points = []
                    for constraint_tuple in self.constraints:
                        shape1, shape2, constraint_type, value = constraint_tuple
                        if constraint_type.lower() == "pointonline":
                            # Check if this line is one of the constrained shapes
                            if shape2 is shape and isinstance(shape1, drawmodel.Point):
                                constrained_points.append(shape1)
                            elif shape1 is shape and isinstance(shape2, drawmodel.Point):
                                constrained_points.append(shape2)

                    if len(constrained_points) >= 2:
                        # Use the first two constrained points to define the line
                        # These points have already been correctly positioned by the solver
                        point1 = constrained_points[0]
                        point2 = constrained_points[1]

                        # Get the direction from these two points
                        dx = point2.coords.x - point1.coords.x
                        dy = point2.coords.y - point1.coords.y
                        length = math.sqrt(dx * dx + dy * dy)

                        if length > 1e-6:
                            # Extend the line beyond the constrained points for visibility
                            extension = 50.0  # Fixed extension in pixels
                            dir_x = dx / length
                            dir_y = dy / length

                            # Extend from point1 backwards and from point2 forwards
                            new_x1 = point1.coords.x - dir_x * extension
                            new_y1 = point1.coords.y - dir_y * extension
                            new_x2 = point2.coords.x + dir_x * extension
                            new_y2 = point2.coords.y + dir_y * extension

                            shape.defining_point_one.coords.x = new_x1
                            shape.defining_point_one.coords.y = new_y1
                            shape.defining_point_two.coords.x = new_x2
                            shape.defining_point_two.coords.y = new_y2

                            print(f"  Updated Line (node {node_id}) from constrained points, extended by {extension}px on each end")
                        else:
                            print(f"  ⚠ Warning: Line (node {node_id}) has degenerate constrained points (same position)")
                    else:
                        print(f"  ⚠ Warning: Line (node {node_id}) has less than 2 constrained points, cannot update")

            print("✓ Successfully updated all shape positions")
            return True

        except Exception as e:
            print(f"✗ Error during solving: {e}")
            import traceback
            traceback.print_exc()
            return False

    def get_graph_nodes(self):
        """Get all nodes in the constraint graph for visualization"""
        if not BINDINGS_AVAILABLE or self.graph_handle is None:
            return []

        try:
            return self.graph_handle.getNodes()
        except Exception as e:
            print(f"Error getting graph nodes: {e}")
            return []

    def get_graph_edges(self):
        """Get all edges in the constraint graph for visualization"""
        if not BINDINGS_AVAILABLE or self.graph_handle is None:
            return []

        try:
            return self.graph_handle.getEdges()
        except Exception as e:
            print(f"Error getting graph edges: {e}")
            return []

    def get_decomposed_subgraphs(self):
        """Get decomposed subgraphs for visualization

        Returns:
            List of SubgraphInfo objects, each containing nodes and edges
        """
        if not BINDINGS_AVAILABLE or self.graph_handle is None:
            return []

        try:
            return self.graph_handle.getDecomposedSubgraphs()
        except Exception as e:
            print(f"Error getting decomposed subgraphs: {e}")
            import traceback
            traceback.print_exc()
            return []

    def remove_shape(self, shape: drawmodel.Drawable):
        """Remove a shape from the constraint graph"""
        # Note: Current ConstraintGraphHandle doesn't support node removal
        # This would require rebuilding the graph
        print("⚠ Warning: Shape removal not implemented in ConstraintGraphHandle")
        print("  Consider rebuilding the graph if needed")

        node_id = self.shape_to_node_id.get(shape)
        if node_id is not None:
            # Remove from shape mapping
            del self.shape_to_node_id[shape]

            # Remove from constraints list
            self.constraints = [
                (s1, s2, constraint_type, value)
                for s1, s2, constraint_type, value in self.constraints
                if s1 != shape and s2 != shape
            ]

            print(f"Removed shape {type(shape).__name__} (node {node_id}) from tracking")
            return True
        return False

    def clear_constraint_graph(self):
        """Clear the entire constraint graph"""
        if BINDINGS_AVAILABLE and self.graph_handle is not None:
            self.graph_handle.clear()

        self.shape_to_node_id.clear()
        self.constraints.clear()
        print("Cleared entire constraint graph")

    def cleanup(self):
        """Cleanup resources to prevent memory leaks"""
        if BINDINGS_AVAILABLE and self.graph_handle is not None:
            self.graph_handle.clear()
            # Explicitly delete the handle to help with cleanup
            del self.graph_handle
            self.graph_handle = None

        self.shape_to_node_id.clear()
        self.constraints.clear()
        self.shape_data_ref = None

    def save_to_file(self, filename: str):
        """Save the current drawing and constraints to a JSON file"""
        try:
            # Serialize all shapes
            shapes_data = []
            shape_to_index = {}  # Map shape object to its index

            for i, shape in enumerate(self.shape_data_ref.shape_buffer):
                shapes_data.append(shape.to_dict())
                shape_to_index[shape] = i

            # Serialize constraints (using shape indices instead of references)
            constraints_data = []
            for shape1, shape2, constraint_type, value in self.constraints:
                if shape1 in shape_to_index and shape2 in shape_to_index:
                    constraints_data.append({
                        "shape1_index": shape_to_index[shape1],
                        "shape2_index": shape_to_index[shape2],
                        "constraint_type": constraint_type,
                        "value": value
                    })

            # Create the full data structure
            save_data = {
                "version": "1.0",
                "shapes": shapes_data,
                "constraints": constraints_data
            }

            # Write to file
            with open(filename, 'w') as f:
                json.dump(save_data, f, indent=2)

            print(f"✓ Saved drawing to {filename}")
            print(f"  Shapes: {len(shapes_data)}, Constraints: {len(constraints_data)}")
            return True

        except Exception as e:
            print(f"✗ Error saving to file: {e}")
            import traceback
            traceback.print_exc()
            return False

    def load_from_file(self, filename: str):
        """Load a drawing and constraints from a JSON file"""
        try:
            # Read the file
            with open(filename, 'r') as f:
                save_data = json.load(f)

            # Clear existing data
            self.shape_data_ref.shape_buffer.clear()
            self.clear_constraint_graph()

            # Deserialize shapes
            shape_objects = []
            for shape_data in save_data["shapes"]:
                shape_type = shape_data["type"]

                if shape_type == "Point":
                    shape = drawmodel.Point.from_dict(shape_data)
                elif shape_type == "Line":
                    shape = drawmodel.Line.from_dict(shape_data)
                elif shape_type == "Circle":
                    shape = drawmodel.Circle.from_dict(shape_data)
                else:
                    print(f"Warning: Unknown shape type {shape_type}")
                    continue

                shape_objects.append(shape)
                self.shape_data_ref.shape_buffer.append(shape)

            # Rebuild constraints and constraint graph
            for constraint_data in save_data["constraints"]:
                shape1_idx = constraint_data["shape1_index"]
                shape2_idx = constraint_data["shape2_index"]
                constraint_type = constraint_data["constraint_type"]
                value = constraint_data["value"]

                if shape1_idx < len(shape_objects) and shape2_idx < len(shape_objects):
                    shape1 = shape_objects[shape1_idx]
                    shape2 = shape_objects[shape2_idx]

                    # Add constraint (this will automatically rebuild the constraint graph)
                    self.add_constraint_between_shapes(
                        shape1, shape2, constraint_type, value
                    )

            print(f"✓ Loaded drawing from {filename}")
            print(f"  Shapes: {len(shape_objects)}, Constraints: {len(save_data['constraints'])}")
            return True

        except Exception as e:
            print(f"✗ Error loading from file: {e}")
            import traceback
            traceback.print_exc()
            return False
