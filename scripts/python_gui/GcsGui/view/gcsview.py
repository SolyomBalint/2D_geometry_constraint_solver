import gi

gi.require_version("Gtk", "3.0")
gi.require_version("Gdk", "3.0")
from gi.repository import Gdk, Gio, GLib, Gtk
from graph_tool.all import *


class ConstraintGraphWidget(Gtk.Box):
    """Widget that displays the real-time constraint graph"""

    def __init__(self, gcs_system):
        super().__init__(orientation=Gtk.Orientation.VERTICAL, spacing=5)
        self.gcs_system = gcs_system

        # Track previous state to detect changes
        self.last_node_count = 0
        self.last_edge_count = 0

        # Store decomposed subgraphs
        self.decomposed_subgraphs = []
        self.showing_decomposed = False

        # Create info label
        self.info_label = Gtk.Label()
        self.info_label.set_markup("<b>Constraint Graph Information</b>")
        self.pack_start(self.info_label, False, False, 5)

        # Create button box
        button_box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=5)

        # Add Decompose button
        self.decompose_button = Gtk.Button.new_with_label("Decompose")
        self.decompose_button.connect("clicked", self.on_decompose_clicked)
        button_box.pack_start(self.decompose_button, False, False, 5)

        # Add Show Original button
        self.show_original_button = Gtk.Button.new_with_label("Show Original")
        self.show_original_button.connect("clicked", self.on_show_original_clicked)
        self.show_original_button.set_sensitive(False)
        button_box.pack_start(self.show_original_button, False, False, 5)

        # Add Rearrange Layout button
        self.rearrange_button = Gtk.Button.new_with_label("Rearrange Layout")
        self.rearrange_button.connect("clicked", self.on_rearrange_clicked)
        button_box.pack_start(self.rearrange_button, False, False, 5)

        self.pack_start(button_box, False, False, 5)

        # Create status labels
        self.nodes_label = Gtk.Label("Nodes: 0")
        self.edges_label = Gtk.Label("Edges: 0")
        self.subgraphs_label = Gtk.Label("Subgraphs: []")

        self.pack_start(self.nodes_label, False, False, 2)
        self.pack_start(self.edges_label, False, False, 2)
        self.pack_start(self.subgraphs_label, False, False, 2)

        # Create graph visualization
        self.graph_widget = None
        self.create_empty_graph_widget()

        # Start periodic update
        GLib.timeout_add(1000, self.update_graph_display)  # Update every second

    def on_decompose_clicked(self, button):
        """Handle decompose button click"""
        if self.gcs_system is None:
            print("✗ No GCS system available")
            return

        print("Decompose button clicked")
        self.decomposed_subgraphs = self.gcs_system.decompose_constraint_graph()

        if len(self.decomposed_subgraphs) > 0:
            self.showing_decomposed = True
            self.show_original_button.set_sensitive(True)
            self.update_decomposed_graph_display()
        else:
            print("✗ No subgraphs returned from decomposition")

    def on_show_original_clicked(self, button):
        """Handle show original button click"""
        self.showing_decomposed = False
        self.show_original_button.set_sensitive(False)
        # Force update of the graph display
        self.last_node_count = -1
        self.last_edge_count = -1

    def on_rearrange_clicked(self, button):
        """Handle rearrange layout button click"""
        print("Rearrange layout button clicked")

        # Force redraw based on current view mode
        if self.showing_decomposed and len(self.decomposed_subgraphs) > 0:
            # Redraw decomposed view
            self.update_decomposed_graph_display()
        else:
            # Force redraw of original graph
            self.last_node_count = -1
            self.last_edge_count = -1
            # Trigger immediate update
            self.update_graph_display()

    def create_empty_graph_widget(self):
        """Create an empty graph widget for initial display"""
        try:
            # Start with a simple label instead of trying to create an empty graph
            self.graph_widget = Gtk.Label("No constraint graph to display")
            self.pack_start(self.graph_widget, True, True, 5)
        except Exception as e:
            print(f"Error creating graph widget: {e}")
            # Fallback to a simple label
            fallback_label = Gtk.Label("Graph visualization unavailable")
            self.pack_start(fallback_label, True, True, 5)

    def update_graph_display(self):
        """Update the graph display with current constraint graph info"""
        if self.gcs_system is None:
            return True  # Continue the timer

        # If showing decomposed graphs, don't update automatically
        if self.showing_decomposed:
            return True

        try:
            # Get current graph info
            info = self.gcs_system.get_constraint_graph_info()

            # Always update status labels
            self.nodes_label.set_text(f"Nodes: {info['nodes']}")
            self.edges_label.set_text(f"Edges: {info['edges']}")
            self.subgraphs_label.set_text(
                f"Subgraph sizes: {info['subgraph_sizes']}"
            )

            # Only update graph visualization if the graph structure has changed
            if (
                info["nodes"] != self.last_node_count
                or info["edges"] != self.last_edge_count
            ):
                print(
                    f"Graph structure changed: nodes {self.last_node_count} -> {info['nodes']}, edges {self.last_edge_count} -> {info['edges']}"
                )

                # Update visualization
                if info["nodes"] > 0:
                    self.update_graph_visualization(info)
                else:
                    # No nodes, show empty message
                    if self.graph_widget is not None:
                        self.remove(self.graph_widget)
                    self.graph_widget = Gtk.Label(
                        "No constraint graph to display"
                    )
                    self.pack_start(self.graph_widget, True, True, 5)
                    self.show_all()

                # Update tracking variables
                self.last_node_count = info["nodes"]
                self.last_edge_count = info["edges"]

        except Exception as e:
            print(f"Error updating graph display: {e}")

        return True  # Continue the timer

    def update_decomposed_graph_display(self):
        """Display decomposed subgraphs as a single disconnected graph"""
        if len(self.decomposed_subgraphs) == 0:
            return

        try:
            # Remove old graph widget
            if self.graph_widget is not None:
                self.remove(self.graph_widget)

            # Create a single combined graph
            combined_graph = self.create_combined_subgraph_visualization()

            if combined_graph is not None:
                self.graph_widget = combined_graph
                self.pack_start(self.graph_widget, True, True, 5)
                self.show_all()
                print(f"✓ Displayed {len(self.decomposed_subgraphs)} decomposed subgraphs in single view")
            else:
                self.graph_widget = Gtk.Label("Error creating combined graph visualization")
                self.pack_start(self.graph_widget, True, True, 5)
                self.show_all()

        except Exception as e:
            print(f"✗ Error displaying decomposed graphs: {e}")
            import traceback
            traceback.print_exc()

    def create_combined_subgraph_visualization(self):
        """Create visualization combining all subgraphs into one disconnected graph"""
        try:
            # Create a single graph for all subgraphs
            combined_graph = Graph(directed=False)

            # Create properties
            vertex_text = combined_graph.new_vertex_property("string")
            vertex_color = combined_graph.new_vertex_property("vector<double>")
            vertex_size = combined_graph.new_vertex_property("double")
            edge_text = combined_graph.new_edge_property("string")

            # Different colors for each subgraph
            subgraph_colors = [
                [0.8, 0.4, 0.2, 1.0],  # Orange
                [0.2, 0.8, 0.4, 1.0],  # Green
                [0.4, 0.2, 0.8, 1.0],  # Purple
                [0.8, 0.2, 0.4, 1.0],  # Pink
                [0.2, 0.4, 0.8, 1.0],  # Blue
                [0.8, 0.8, 0.2, 1.0],  # Yellow
            ]

            all_vertices = []

            # Process each subgraph
            for subgraph_idx, subgraph in enumerate(self.decomposed_subgraphs):
                nodes = subgraph.get_nodes()
                edges = subgraph.get_edges()

                if len(nodes) == 0:
                    continue

                # Map for this subgraph's nodes
                vertex_id_map = {}
                color = subgraph_colors[subgraph_idx % len(subgraph_colors)]

                # Add vertices for this subgraph
                for i, node in enumerate(nodes):
                    if node is not None:
                        v = combined_graph.add_vertex()
                        all_vertices.append(v)
                        vertex_id_map[i] = v
                        vertex_text[v] = f"{node.type}"
                        vertex_color[v] = color
                        vertex_size[v] = 50

                # Add edges for this subgraph
                for node_id1, node_id2, constraint in edges:
                    if node_id1 in vertex_id_map and node_id2 in vertex_id_map:
                        v1 = vertex_id_map[node_id1]
                        v2 = vertex_id_map[node_id2]
                        e = combined_graph.add_edge(v1, v2)
                        edge_text[e] = f"{constraint.type}: {constraint.value:.1f}"

            # Create layout
            if len(all_vertices) > 0:
                try:
                    # SFDP layout handles disconnected components well
                    pos = sfdp_layout(combined_graph, K=1.0, max_iter=100)
                except Exception:
                    # Fallback to manual positioning
                    pos = combined_graph.new_vertex_property("vector<double>")
                    import math
                    for i, v in enumerate(combined_graph.vertices()):
                        angle = 2 * math.pi * i / len(all_vertices)
                        pos[v] = [50 * math.cos(angle), 50 * math.sin(angle)]

                # Create info label
                info_text = f"Decomposed into {len(self.decomposed_subgraphs)} subgraphs: "
                info_text += ", ".join([f"SG{i+1}({sg.get_node_count()}N/{sg.get_edge_count()}E)"
                                       for i, sg in enumerate(self.decomposed_subgraphs)])

                # Create a VBox to hold label and graph
                vbox = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=5)

                info_label = Gtk.Label()
                info_label.set_markup(f"<b>{info_text}</b>")
                vbox.pack_start(info_label, False, False, 5)

                # Create GraphWidget
                graph_widget = GraphWidget(
                    combined_graph,
                    pos,
                    vertex_text=vertex_text,
                    vertex_fill_color=vertex_color,
                    vertex_size=vertex_size,
                    edge_text=edge_text,
                    vertex_font_size=15,
                    edge_font_size=12,
                )
                vbox.pack_start(graph_widget, True, True, 5)

                return vbox
            else:
                return Gtk.Label("No vertices in decomposed subgraphs")

        except Exception as e:
            print(f"Error creating combined subgraph visualization: {e}")
            import traceback
            traceback.print_exc()
            return Gtk.Label(f"Error visualizing combined subgraphs: {e}")

    def create_subgraph_visualization(self, subgraph):
        """Create visualization for a single subgraph"""
        try:
            nodes = subgraph.get_nodes()
            edges = subgraph.get_edges()

            if len(nodes) == 0:
                return Gtk.Label("Empty subgraph")

            # Create a new graph for visualization
            indirected_graph = Graph(directed=False)

            # Create vertex properties for labels and styling
            vertex_text = indirected_graph.new_vertex_property("string")
            vertex_color = indirected_graph.new_vertex_property("vector<double>")
            vertex_size = indirected_graph.new_vertex_property("double")
            edge_text = indirected_graph.new_edge_property("string")

            # Add vertices
            vertices = []
            vertex_id_map = {}

            for i, node in enumerate(nodes):
                if node is not None:
                    v = indirected_graph.add_vertex()
                    vertices.append(v)
                    vertex_id_map[i] = v
                    vertex_text[v] = node.type
                    vertex_color[v] = [0.8, 0.4, 0.2, 1.0]  # Orange color for subgraphs
                    vertex_size[v] = 40

            # Add edges
            for node_id1, node_id2, constraint in edges:
                if node_id1 in vertex_id_map and node_id2 in vertex_id_map:
                    v1 = vertex_id_map[node_id1]
                    v2 = vertex_id_map[node_id2]
                    e = indirected_graph.add_edge(v1, v2)
                    edge_text[e] = f"{constraint.type}: {constraint.value:.1f}"

            # Create layout
            if len(vertices) > 0:
                try:
                    pos = sfdp_layout(indirected_graph, K=0.5, max_iter=50)
                except Exception:
                    # Fallback to manual circular positioning
                    pos = indirected_graph.new_vertex_property("vector<double>")
                    import math
                    for i, v in enumerate(indirected_graph.vertices()):
                        angle = 2 * math.pi * i / len(vertices)
                        pos[v] = [30 * math.cos(angle), 30 * math.sin(angle)]

                # Create GraphWidget with reduced size
                graph_widget = GraphWidget(
                    indirected_graph,
                    pos,
                    vertex_text=vertex_text,
                    vertex_fill_color=vertex_color,
                    vertex_size=vertex_size,
                    edge_text=edge_text,
                    vertex_font_size=12,
                    edge_font_size=12,
                )
                graph_widget.set_size_request(-1, 300)  # Set fixed height
                return graph_widget
            else:
                return Gtk.Label("Empty subgraph")

        except Exception as e:
            print(f"Error creating subgraph visualization: {e}")
            return Gtk.Label(f"Error visualizing subgraph: {e}")

    def update_graph_visualization(self, info):
        """Update the actual graph visualization"""
        try:
            # Get the actual constraint graph data
            constraint_graph = self.gcs_system.constraint_graph
            nodes = constraint_graph.get_nodes()
            edges = constraint_graph.get_edges()

            # Create a new graph for visualization
            indirected_graph = Graph(directed=False)

            # Create vertex properties for labels and styling
            vertex_text = indirected_graph.new_vertex_property("string")
            vertex_color = indirected_graph.new_vertex_property(
                "vector<double>"
            )
            vertex_size = indirected_graph.new_vertex_property("double")

            # Create edge properties for labels
            edge_text = indirected_graph.new_edge_property("string")

            # Add vertices (nodes) with actual data
            vertices = []
            vertex_id_map = (
                {}
            )  # Map from constraint graph node IDs to graph vertices

            for i, node in enumerate(nodes):
                if node is not None:  # Skip removed nodes
                    v = indirected_graph.add_vertex()
                    vertices.append(v)
                    vertex_id_map[i] = v

                    # Set node label based on element type
                    vertex_text[v] = node.type

                    # Set node color (blue) and size
                    vertex_color[v] = [0.2, 0.4, 0.8, 1.0]  # Blue color (RGBA)
                    vertex_size[v] = 50  # Bigger size

            # Add edges (constraints) with actual data
            for node_id1, node_id2, constraint in edges:
                if node_id1 in vertex_id_map and node_id2 in vertex_id_map:
                    v1 = vertex_id_map[node_id1]
                    v2 = vertex_id_map[node_id2]
                    e = indirected_graph.add_edge(v1, v2)

                    # Set edge label with constraint type and value
                    edge_text[e] = f"{constraint.type}: {constraint.value:.1f}"

            # Create layout with error handling
            if len(vertices) > 0:
                # Use sfdp_layout with conservative settings
                try:
                    pos = sfdp_layout(indirected_graph, K=0.5, max_iter=50)

                    # Validate positions to avoid infinity errors
                    valid_positions = True
                    for v in indirected_graph.vertices():
                        if not (
                            abs(pos[v][0]) < float("inf")
                            and abs(pos[v][1]) < float("inf")
                        ):
                            valid_positions = False
                            break

                    if not valid_positions:
                        # Fallback to manual positioning
                        pos = indirected_graph.new_vertex_property(
                            "vector<double>"
                        )
                        import math

                        for i, v in enumerate(indirected_graph.vertices()):
                            angle = 2 * math.pi * i / len(vertices)
                            pos[v] = [
                                50 * math.cos(angle),
                                50 * math.sin(angle),
                            ]

                except Exception as layout_error:
                    print(
                        f"Layout error: {layout_error}, using manual positioning"
                    )
                    # Manual circular positioning as fallback
                    pos = indirected_graph.new_vertex_property("vector<double>")
                    import math

                    for i, v in enumerate(indirected_graph.vertices()):
                        angle = 2 * math.pi * i / len(vertices)
                        pos[v] = [50 * math.cos(angle), 50 * math.sin(angle)]

                # Remove old graph widget and create new one
                if self.graph_widget is not None:
                    self.remove(self.graph_widget)

                # Create GraphWidget with styling properties
                self.graph_widget = GraphWidget(
                    indirected_graph,
                    pos,
                    vertex_text=vertex_text,
                    vertex_fill_color=vertex_color,
                    vertex_size=vertex_size,
                    edge_text=edge_text,
                    vertex_font_size=15,
                    edge_font_size=15,
                )
                self.pack_start(self.graph_widget, True, True, 5)
                self.show_all()
            else:
                # No vertices, show message
                if self.graph_widget is not None:
                    self.remove(self.graph_widget)
                self.graph_widget = Gtk.Label("No nodes in constraint graph")
                self.pack_start(self.graph_widget, True, True, 5)
                self.show_all()

        except Exception as e:
            print(f"Error updating graph visualization: {e}")
            # Fallback to text display
            if self.graph_widget is not None:
                self.remove(self.graph_widget)
            self.graph_widget = Gtk.Label(
                f"Graph: {info['nodes']} nodes, {info['edges']} edges"
            )
            self.pack_start(self.graph_widget, True, True, 5)
            self.show_all()


def create_constraint_graph_view(gcs_system):
    """Create the constraint graph view widget"""
    return ConstraintGraphWidget(gcs_system)


# TODO - Legacy function for compatibility
def own_small_example():
    indirected_graph = Graph(directed=False)
    vertice1 = indirected_graph.add_vertex()
    vertice2 = indirected_graph.add_vertex()
    vertice3 = indirected_graph.add_vertex()

    edge1 = indirected_graph.add_edge(vertice1, vertice2)
    edge2 = indirected_graph.add_edge(vertice1, vertice3)
    edge3 = indirected_graph.add_edge(vertice3, vertice2)

    K = 0.5
    pos = sfdp_layout(indirected_graph, K=K)
    return GraphWidget(indirected_graph, pos)
