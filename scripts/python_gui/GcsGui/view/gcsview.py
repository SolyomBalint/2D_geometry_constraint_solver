import gi

gi.require_version("Gtk", "3.0")
gi.require_version("Gdk", "3.0")
from gi.repository import Gdk, Gio, Gtk
from graph_tool.all import *


# TODO
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
