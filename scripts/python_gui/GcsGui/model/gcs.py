import os
import sys
import json

# Add path to C++ bindings
sys.path.append(os.path.join(os.path.dirname(__file__), '../../../../build/Debug'))

BINDINGS_AVAILABLE = False
scg = None

try:
    import simple_constraint_graph_binding as scg
    BINDINGS_AVAILABLE = True
    print("✓ Successfully imported simple_constraint_graph_binding")
except ImportError as e:
    print(f"✗ Failed to import simple_constraint_graph_binding: {e}")
    print("  Falling back to Python-only implementation")
    BINDINGS_AVAILABLE = False

from . import drawmodel


class SimpleElement:
    """Python representation of SimpleElement for constraint graph"""

    def __init__(self, element_type, data):
        self.type = element_type  # 'Point', 'Circle', 'Line'
        self.data = data  # List of coordinate/parameter data

    @classmethod
    def create_point(cls, x, y):
        return cls("Point", [x, y])

    @classmethod
    def create_circle(cls, x, y, radius):
        return cls("Circle", [x, y, radius])

    @classmethod
    def create_line(cls, r0_x, r0_y, v_x, v_y):
        return cls("Line", [r0_x, r0_y, v_x, v_y])


class SimpleConstraint:
    """Python representation of SimpleConstraint for constraint graph"""

    def __init__(self, constraint_type, value):
        self.type = constraint_type  # 'Distance', 'Tangency'
        self.value = value

    @classmethod
    def create_distance(cls, value):
        return cls("Distance", value)

    @classmethod
    def create_tangency(cls, value):
        return cls("Tangency", value)


class SimpleConstraintGraph:
    """Python-based constraint graph that can be converted to C++ later"""

    def __init__(self):
        self.nodes = []  # List of SimpleElement
        self.edges = []  # List of (node_id1, node_id2, SimpleConstraint)
        self.next_node_id = 0

    def add_node(self, element):
        """Add a node and return its ID"""
        self.nodes.append(element)
        node_id = self.next_node_id
        self.next_node_id += 1
        return node_id

    def add_edge(self, node_id1, node_id2, constraint):
        """Add an edge between two nodes"""
        if node_id1 < len(self.nodes) and node_id2 < len(self.nodes):
            self.edges.append((node_id1, node_id2, constraint))
            return len(self.edges) - 1  # Return edge ID
        return -1

    def get_node_count(self):
        return len([node for node in self.nodes if node is not None])

    def get_edge_count(self):
        return len(self.edges)

    def get_nodes(self):
        return self.nodes

    def get_edges(self):
        return self.edges

    def remove_node(self, node_id):
        """Remove a node and all edges connected to it"""
        if 0 <= node_id < len(self.nodes):
            # Remove all edges that connect to this node
            self.edges = [
                (id1, id2, constraint)
                for id1, id2, constraint in self.edges
                if id1 != node_id and id2 != node_id
            ]

            # Mark the node as removed (we can't actually remove it due to ID mapping)
            # Instead, we'll set it to None and handle this in get_node_count
            self.nodes[node_id] = None
            return True
        return False

    def clear(self):
        """Clear all nodes and edges"""
        self.nodes.clear()
        self.edges.clear()
        self.next_node_id = 0


class GeometricConstraintSystem:
    def __init__(self, shape_data):
        self.shape_data_ref = shape_data
        # Create a Python-based SimpleConstraintGraph
        self.constraint_graph = SimpleConstraintGraph()

        # Map drawable shapes to constraint graph node IDs
        self.shape_to_node_id = {}

        # Track constraints between shapes
        self.constraints = (
            []
        )  # List of (shape1, shape2, constraint_type, value) tuples

    def add_shape_as_node(self, shape: drawmodel.Drawable):
        """Add a drawable shape as a node in the constraint graph"""
        try:
            # Convert drawable shape to SimpleElement
            if isinstance(shape, drawmodel.Point):
                element = SimpleElement.create_point(
                    shape.coords.x, shape.coords.y
                )
            elif isinstance(shape, drawmodel.Circle):
                element = SimpleElement.create_circle(
                    shape.center.x, shape.center.y, shape.radius
                )
            elif isinstance(shape, drawmodel.Line):
                # For lines, we use the direction vector representation
                # r0 = starting point, v = direction vector
                r0_x = shape.defining_point_one.coords.x
                r0_y = shape.defining_point_one.coords.y
                v_x = shape.defining_point_two.coords.x - r0_x
                v_y = shape.defining_point_two.coords.y - r0_y
                element = SimpleElement.create_line(r0_x, r0_y, v_x, v_y)
            else:
                print(f"Warning: Unknown shape type {type(shape)}")
                return None

            # Add node to constraint graph
            node_id = self.constraint_graph.add_node(element)
            self.shape_to_node_id[shape] = node_id

            print(f"Added shape {type(shape).__name__} as node {node_id}")
            return node_id

        except Exception as e:
            print(f"Error adding shape as node: {e}")
            return None

    def add_constraint_between_shapes(
        self,
        shape1: drawmodel.Drawable,
        shape2: drawmodel.Drawable,
        constraint_type: str,
        value: float,
    ):
        """Add a constraint between two shapes"""
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
            # Create constraint
            if constraint_type.lower() == "distance":
                constraint = SimpleConstraint.create_distance(value)
            elif constraint_type.lower() == "tangency":
                constraint = SimpleConstraint.create_tangency(value)
            else:
                print(f"Unknown constraint type: {constraint_type}")
                return False

            # Add edge to constraint graph
            edge_id = self.constraint_graph.add_edge(
                node1_id, node2_id, constraint
            )

            # Track the constraint
            self.constraints.append((shape1, shape2, constraint_type, value))

            print(
                f"Added {constraint_type} constraint (value: {value}) between nodes {node1_id} and {node2_id}, edge ID: {edge_id}"
            )
            return True

        except Exception as e:
            print(f"Error adding constraint: {e}")
            return False

    def get_constraint_graph_info(self):
        """Get information about the current constraint graph"""
        try:
            node_count = self.constraint_graph.get_node_count()
            edge_count = self.constraint_graph.get_edge_count()

            # For now, we don't do complex well-constrained analysis in Python
            # This will be done when we convert to C++ for solving
            well_constrained = False
            subgraph_sizes = []

            # Simple heuristic: 2D constraints generally need 2*n - 3 constraints for n points
            # This is a rough approximation for display purposes
            if node_count > 2:
                expected_constraints = 2 * node_count - 3
                well_constrained = edge_count >= expected_constraints

            return {
                "nodes": node_count,
                "edges": edge_count,
                "well_constrained": well_constrained,
                "subgraph_sizes": [node_count] if node_count > 0 else [],
            }
        except Exception as e:
            print(f"Error getting constraint graph info: {e}")
            return {
                "nodes": 0,
                "edges": 0,
                "well_constrained": False,
                "subgraph_sizes": [],
            }

    def solve_constraint_system(self):
        """Solve the constraint system and return updated positions"""
        # For now, we just return the current positions without solving
        # Real solving will be done when converted to C++
        try:
            if self.constraint_graph.get_node_count() == 0:
                return []

            print(
                f"Mock solving constraint system with {self.constraint_graph.get_node_count()} nodes and {self.constraint_graph.get_edge_count()} edges"
            )
            print(
                "Note: Real constraint solving will be implemented when converting to C++"
            )

            # Return mock solved positions (just current positions)
            positions = []
            for node in self.constraint_graph.get_nodes():
                if node.type == "Point":
                    positions.append([node.data[0], node.data[1]])
                elif node.type == "Circle":
                    positions.append(
                        [node.data[0], node.data[1]]
                    )  # Center position
                elif node.type == "Line":
                    positions.append(
                        [node.data[0], node.data[1]]
                    )  # Start position

            return positions

        except Exception as e:
            print(f"Error in mock solving constraint system: {e}")
            return None

    def remove_shape(self, shape: drawmodel.Drawable):
        """Remove a shape from the constraint graph"""
        node_id = self.shape_to_node_id.get(shape)
        if node_id is not None:
            # Remove the node and its edges from the constraint graph
            self.constraint_graph.remove_node(node_id)

            # Remove from shape mapping
            del self.shape_to_node_id[shape]

            # Remove from constraints list
            self.constraints = [
                (s1, s2, constraint_type, value)
                for s1, s2, constraint_type, value in self.constraints
                if s1 != shape and s2 != shape
            ]

            print(
                f"Removed shape {type(shape).__name__} (node {node_id}) from constraint graph"
            )
            return True
        return False

    def clear_constraint_graph(self):
        """Clear the entire constraint graph"""
        self.constraint_graph.clear()
        self.shape_to_node_id.clear()
        self.constraints.clear()
        print("Cleared entire constraint graph")

    def convert_to_cpp_graph(self):
        """Convert Python SimpleConstraintGraph to C++ SimpleConstraintGraph"""
        if not BINDINGS_AVAILABLE or scg is None:
            print("✗ C++ bindings not available for conversion")
            return None

        try:
            # Create a C++ SimpleConstraintGraph
            cpp_graph = scg.SimpleConstraintGraph()

            # Convert nodes
            node_mapping = {}  # Map Python node IDs to C++ node IDs
            for i, py_node in enumerate(self.constraint_graph.get_nodes()):
                if py_node is None:
                    continue

                # Create C++ element based on type
                if py_node.type == "Point":
                    cpp_element = scg.SimpleElement.createPoint(
                        py_node.data[0], py_node.data[1]
                    )
                elif py_node.type == "Circle":
                    cpp_element = scg.SimpleElement.createCircle(
                        py_node.data[0], py_node.data[1], py_node.data[2]
                    )
                elif py_node.type == "Line":
                    cpp_element = scg.SimpleElement.createLine(
                        py_node.data[0], py_node.data[1],
                        py_node.data[2], py_node.data[3]
                    )
                else:
                    continue

                cpp_node_id = cpp_graph.addNode(cpp_element)
                node_mapping[i] = cpp_node_id

            # Convert edges
            for node_id1, node_id2, py_constraint in self.constraint_graph.get_edges():
                if node_id1 not in node_mapping or node_id2 not in node_mapping:
                    continue

                # Create C++ constraint based on type
                if py_constraint.type == "Distance":
                    cpp_constraint = scg.SimpleConstraint.createDistance(
                        py_constraint.value
                    )
                elif py_constraint.type == "Tangency":
                    cpp_constraint = scg.SimpleConstraint.createTangency(
                        py_constraint.value
                    )
                else:
                    continue

                cpp_graph.addEdge(
                    node_mapping[node_id1],
                    node_mapping[node_id2],
                    cpp_constraint
                )

            print(f"✓ Converted Python graph to C++ graph: {cpp_graph.getNodeCount()} nodes, {cpp_graph.getEdgeCount()} edges")
            return cpp_graph

        except Exception as e:
            print(f"✗ Error converting Python graph to C++ graph: {e}")
            return None

    def decompose_constraint_graph(self):
        """Decompose the constraint graph using C++ binding and return subgraphs"""
        if not BINDINGS_AVAILABLE or scg is None:
            print("✗ C++ bindings not available for decomposition")
            return []

        try:
            # Convert Python graph to C++ graph
            cpp_graph = self.convert_to_cpp_graph()
            if cpp_graph is None:
                return []

            # Call C++ decomposition function
            print("Calling C++ decomposeConstraintGraph...")
            cpp_subgraphs = scg.decomposeConstraintGraph(cpp_graph)

            print(f"✓ Decomposed into {len(cpp_subgraphs)} subgraphs")

            # Convert C++ subgraphs back to Python representation
            python_subgraphs = []
            for i, cpp_subgraph in enumerate(cpp_subgraphs):
                py_subgraph = SimpleConstraintGraph()

                # Convert nodes
                cpp_nodes = cpp_subgraph.getNodes()
                for cpp_node in cpp_nodes:
                    py_node = SimpleElement(cpp_node.type.name, list(cpp_node.data))
                    py_subgraph.add_node(py_node)

                # Convert edges
                cpp_edges = cpp_subgraph.getEdges()
                for cpp_edge in cpp_edges:
                    py_constraint = SimpleConstraint(
                        cpp_edge.constraint.type.name,
                        cpp_edge.constraint.value
                    )
                    py_subgraph.add_edge(
                        cpp_edge.nodeId1,
                        cpp_edge.nodeId2,
                        py_constraint
                    )

                python_subgraphs.append(py_subgraph)
                print(f"  Subgraph {i}: {py_subgraph.get_node_count()} nodes, {py_subgraph.get_edge_count()} edges")

            return python_subgraphs

        except Exception as e:
            print(f"✗ Error during decomposition: {e}")
            import traceback
            traceback.print_exc()
            return []

    def solve_constraint_graph(self, canvas_center_x=None, canvas_center_y=None):
        """Solve the constraint graph and update shape positions

        Args:
            canvas_center_x: X coordinate of canvas center (for translating solved positions)
            canvas_center_y: Y coordinate of canvas center (for translating solved positions)
        """
        if not BINDINGS_AVAILABLE or scg is None:
            print("✗ C++ bindings not available for solving")
            return False

        try:
            # Convert Python graph to C++ graph
            cpp_graph = self.convert_to_cpp_graph()
            if cpp_graph is None:
                print("✗ Failed to convert graph")
                return False

            # Call C++ solver function
            print("Calling C++ solveSimpleConstraintGraph...")
            solved_positions = scg.solveSimpleConstraintGraph(cpp_graph)

            print(f"✓ Solver returned {len(solved_positions)} positions")

            # Calculate translation offset
            # Solver starts from (0,0), we want to move it to canvas center
            offset_x = canvas_center_x if canvas_center_x is not None else 0
            offset_y = canvas_center_y if canvas_center_y is not None else 0

            print(f"Translating solved positions by offset ({offset_x:.2f}, {offset_y:.2f})")

            # Update the shapes with solved positions (translated to canvas center)
            node_id_to_shape = {node_id: shape for shape, node_id in self.shape_to_node_id.items()}

            for node_id, position in enumerate(solved_positions):
                if node_id in node_id_to_shape:
                    shape = node_id_to_shape[node_id]

                    # Update shape based on type
                    if isinstance(shape, drawmodel.Point):
                        if len(position) >= 2:
                            shape.coords.x = position[0] + offset_x
                            shape.coords.y = position[1] + offset_y
                            print(f"  Updated Point (node {node_id}) to ({shape.coords.x:.2f}, {shape.coords.y:.2f})")

                    elif isinstance(shape, drawmodel.Circle):
                        if len(position) >= 2:
                            shape.center.x = position[0] + offset_x
                            shape.center.y = position[1] + offset_y
                            print(f"  Updated Circle (node {node_id}) center to ({shape.center.x:.2f}, {shape.center.y:.2f})")

                    elif isinstance(shape, drawmodel.Line):
                        if len(position) >= 4:
                            # Update both defining points of the line
                            # Line is defined by two points, but solver returns r0 and direction vector
                            # We need to update the point positions
                            shape.defining_point_one.coords.x = position[0] + offset_x
                            shape.defining_point_one.coords.y = position[1] + offset_y
                            shape.defining_point_two.coords.x = position[0] + position[2] + offset_x
                            shape.defining_point_two.coords.y = position[1] + position[3] + offset_y
                            print(f"  Updated Line (node {node_id})")

            print("✓ Successfully updated all shape positions")
            return True

        except Exception as e:
            print(f"✗ Error during solving: {e}")
            import traceback
            traceback.print_exc()
            return False

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
