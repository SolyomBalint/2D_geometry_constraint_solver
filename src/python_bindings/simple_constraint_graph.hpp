#ifndef SIMPLE_CONSTRAINT_GRAPH_HPP
#define SIMPLE_CONSTRAINT_GRAPH_HPP

#include <vector>
#include <string>

namespace SimpleCG {

enum class ElementType { Point, FixedRadiusCircle, Line };

enum class ConstraintType { Distance, Tangency };

struct SimpleElement {
    ElementType type;
    double data[4]; // Flexible data storage: [x,y] for Point, [x,y,radius,_]
                    // for Circle, [r0_x,r0_y,v_x,v_y] for Line

    SimpleElement()
        : type(ElementType::Point)
        , data { 0.0, 0.0, 0.0, 0.0 }
    {
    }

    SimpleElement(
        ElementType t, double d1, double d2, double d3 = 0.0, double d4 = 0.0)
        : type(t)
        , data { d1, d2, d3, d4 }
    {
    }

    // Factory methods for type safety
    static SimpleElement createPoint(double x, double y)
    {
        return SimpleElement(ElementType::Point, x, y);
    }

    static SimpleElement createCircle(double x, double y, double radius)
    {
        return SimpleElement(ElementType::FixedRadiusCircle, x, y, radius);
    }

    static SimpleElement createLine(
        double r0_x, double r0_y, double v_x, double v_y)
    {
        return SimpleElement(ElementType::Line, r0_x, r0_y, v_x, v_y);
    }

    std::string toString() const;
};

struct SimpleConstraint {
    ConstraintType type;
    double value;

    SimpleConstraint()
        : type(ConstraintType::Distance)
        , value(0.0)
    {
    }

    SimpleConstraint(ConstraintType t, double v)
        : type(t)
        , value(v)
    {
    }

    // Factory methods
    static SimpleConstraint createDistance(double distance)
    {
        return SimpleConstraint(ConstraintType::Distance, distance);
    }

    static SimpleConstraint createTangency(double angle)
    {
        return SimpleConstraint(ConstraintType::Tangency, angle);
    }

    std::string toString() const;
};

struct SimpleEdge {
    int nodeId1;
    int nodeId2;
    SimpleConstraint constraint;

    SimpleEdge()
        : nodeId1(-1)
        , nodeId2(-1)
    {
    }

    SimpleEdge(int n1, int n2, const SimpleConstraint& c)
        : nodeId1(n1)
        , nodeId2(n2)
        , constraint(c)
    {
    }
};

class SimpleConstraintGraph {
private:
    std::vector<SimpleElement> nodes;
    std::vector<SimpleEdge> edges;

public:
    SimpleConstraintGraph() = default;

    // Node operations
    int addNode(const SimpleElement& element);
    std::size_t getNodeCount() const { return nodes.size(); }
    const SimpleElement& getNode(int nodeId) const;
    const std::vector<SimpleElement>& getNodes() const { return nodes; }

    // Edge operations
    int addEdge(int nodeId1, int nodeId2, const SimpleConstraint& constraint);
    std::size_t getEdgeCount() const { return edges.size(); }
    const SimpleEdge& getEdge(int edgeId) const;
    const std::vector<SimpleEdge>& getEdges() const { return edges; }

    // Utility
    void clear();
    std::string toString() const;
};

} // namespace SimpleCG

#endif // SIMPLE_CONSTRAINT_GRAPH_HPP
