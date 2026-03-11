#include "solving/bottom_up_plan_solver.hpp"

// General STD/STL headers
#include <array>
#include <cmath>
#include <memory>
#include <optional>
#include <unordered_map>

// Custom headers
#include "solving/bottom_up/merge3_fallback_solver.hpp"
#include "solving/bottom_up/merge3_llp_solver.hpp"
#include "solving/bottom_up/merge3_lpp_solver.hpp"
#include "solving/bottom_up/merge3_pll_solver.hpp"
#include "solving/bottom_up/merge3_ppp_solver.hpp"
#include "solving/bottom_up/merge3_solver_common.hpp"
#include "solving/component_solver.hpp"
#include <gcs/model/constraints.hpp>
#include <gcs/model/elements.hpp>

namespace Gcs {

namespace {

    using Solvers::BottomUp::ClusterPose;
    using Solvers::BottomUp::ElementPose;
    using Solvers::BottomUp::LinePose;
    using Solvers::BottomUp::PointPose;

    [[nodiscard]] std::optional<ConstraintGraph::EdgeIdType> edgeBetween(
        const ConstraintGraph& graph, ConstraintGraph::NodeIdType first,
        ConstraintGraph::NodeIdType second)
    {
        const auto edge = graph.getGraph().getEdgeBetween(first, second);
        if (!edge.has_value()) {
            return std::nullopt;
        }

        return edge.value();
    }

    [[nodiscard]] std::optional<ClusterPose> solveEdgePrimitive(
        const ConstraintGraph& sourceGraph,
        const std::array<ConstraintGraph::NodeIdType, 2>& elements)
    {
        const auto firstElement = sourceGraph.getElement(elements[0]);
        const auto secondElement = sourceGraph.getElement(elements[1]);
        if (!firstElement || !secondElement) {
            return std::nullopt;
        }

        const auto edgeId = edgeBetween(sourceGraph, elements[0], elements[1]);
        if (!edgeId.has_value() || sourceGraph.isVirtualEdge(edgeId.value())) {
            return std::nullopt;
        }

        const auto constraint
            = sourceGraph.getConstraintForEdge(edgeId.value());
        if (!constraint) {
            return std::nullopt;
        }

        ClusterPose pose;

        if (firstElement->isElementType<Point>()
            && secondElement->isElementType<Point>()
            && constraint->isConstraintType<DistanceConstraint>()) {
            const double distance = constraint->getConstraintValue().value();

            pose.emplace(
                elements[0], PointPose { .position = Eigen::Vector2d::Zero() });
            pose.emplace(elements[1],
                PointPose {
                    .position = Eigen::Vector2d { distance, 0.0 },
                });
            return pose;
        }

        if (constraint->isConstraintType<DistanceConstraint>()
            && ((firstElement->isElementType<Point>()
                    && secondElement->isElementType<Line>())
                || (firstElement->isElementType<Line>()
                    && secondElement->isElementType<Point>()))) {
            const bool firstIsPoint = firstElement->isElementType<Point>();

            const auto& pointElement
                = firstIsPoint ? firstElement : secondElement;
            const auto& lineElement
                = firstIsPoint ? secondElement : firstElement;

            const auto& pointData = pointElement->getElement<Point>();
            const auto& lineData = lineElement->getElement<Line>();

            const double distance = constraint->getConstraintValue().value();
            const double signedCanvasDistance = Solvers::signedDistanceToLine(
                pointData.canvasPosition, lineData.canvasP1, lineData.canvasP2);
            const double sign = (signedCanvasDistance < 0.0) ? -1.0 : 1.0;
            const double y = sign * distance;

            const double lineLength
                = Solvers::BottomUp::safeCanvasLineLength(lineData);
            const LinePose solvedLine {
                .p1 = Eigen::Vector2d { -lineLength / 2.0, y },
                .p2 = Eigen::Vector2d { lineLength / 2.0, y },
            };

            const PointPose solvedPoint {
                .position = Eigen::Vector2d::Zero(),
            };

            if (firstIsPoint) {
                pose.emplace(elements[0], solvedPoint);
                pose.emplace(elements[1], solvedLine);
            } else {
                pose.emplace(elements[0], solvedLine);
                pose.emplace(elements[1], solvedPoint);
            }

            return pose;
        }

        if (firstElement->isElementType<Line>()
            && secondElement->isElementType<Line>()
            && constraint->isConstraintType<AngleConstraint>()) {
            const auto& firstLineData = firstElement->getElement<Line>();
            const auto& secondLineData = secondElement->getElement<Line>();
            const auto* angleConstraint
                = constraint->getConstraintAs<AngleConstraint>();
            if (angleConstraint == nullptr) {
                return std::nullopt;
            }

            const double firstLength
                = Solvers::BottomUp::safeCanvasLineLength(firstLineData);
            const double secondLength
                = Solvers::BottomUp::safeCanvasLineLength(secondLineData);

            Eigen::Vector2d canvasSecondDirection
                = secondLineData.canvasP2 - secondLineData.canvasP1;
            if (angleConstraint->flipOrientation) {
                canvasSecondDirection = -canvasSecondDirection;
            }

            const Eigen::Vector2d canvasFirstDirection
                = firstLineData.canvasP2 - firstLineData.canvasP1;
            const double cross
                = canvasFirstDirection.x() * canvasSecondDirection.y()
                - canvasFirstDirection.y() * canvasSecondDirection.x();

            const double angle = (cross < 0.0) ? -angleConstraint->angle
                                               : angleConstraint->angle;

            const Eigen::Vector2d secondDirection {
                std::cos(angle),
                std::sin(angle),
            };

            pose.emplace(elements[0], LinePose {
                                       .p1 = Eigen::Vector2d {
                                           -firstLength / 2.0,
                                           0.0,
                                       },
                                       .p2 = Eigen::Vector2d {
                                           firstLength / 2.0,
                                           0.0,
                                       },
                                   });
            pose.emplace(elements[1],
                LinePose {
                    .p1 = -secondDirection * secondLength / 2.0,
                    .p2 = secondDirection * secondLength / 2.0,
                });
            return pose;
        }

        return std::nullopt;
    }

    [[nodiscard]] std::optional<ConstraintGraph> buildPrimitiveSubgraph(
        const ConstraintGraph& sourceGraph,
        std::span<const ConstraintGraph::NodeIdType> primitiveElements,
        std::unordered_map<ConstraintGraph::NodeIdType,
            ConstraintGraph::NodeIdType>& sourceToLocalNode)
    {
        ConstraintGraph primitiveGraph;
        sourceToLocalNode.clear();

        for (const auto& sourceNode : primitiveElements) {
            if (!sourceGraph.getGraph().hasNode(sourceNode)) {
                return std::nullopt;
            }

            const auto localNode = primitiveGraph.getGraph().addNode();
            sourceToLocalNode.emplace(sourceNode, localNode);

            const auto sourceElement = sourceGraph.getElement(sourceNode);
            if (!sourceElement) {
                return std::nullopt;
            }

            auto clonedElement = std::make_shared<Element>(*sourceElement);
            (void)primitiveGraph.addElement(
                localNode, std::move(clonedElement));
        }

        for (std::size_t firstIndex = 0; firstIndex < primitiveElements.size();
            ++firstIndex) {
            for (std::size_t secondIndex = firstIndex + 1;
                secondIndex < primitiveElements.size(); ++secondIndex) {
                const auto sourceEdge
                    = edgeBetween(sourceGraph, primitiveElements[firstIndex],
                        primitiveElements[secondIndex]);
                if (!sourceEdge.has_value()) {
                    continue;
                }

                const auto localSource
                    = sourceToLocalNode.at(primitiveElements[firstIndex]);
                const auto localTarget
                    = sourceToLocalNode.at(primitiveElements[secondIndex]);

                if (sourceGraph.isVirtualEdge(sourceEdge.value())) {
                    (void)primitiveGraph.addVirtualEdge(
                        localSource, localTarget);
                    continue;
                }

                const auto localEdge = primitiveGraph.getGraph().addEdge(
                    localSource, localTarget);
                if (!localEdge.has_value()) {
                    return std::nullopt;
                }

                const auto sourceConstraint
                    = sourceGraph.getConstraintForEdge(sourceEdge.value());
                if (!sourceConstraint) {
                    continue;
                }

                auto clonedConstraint
                    = std::make_shared<Constraint>(*sourceConstraint);
                (void)primitiveGraph.addConstraint(
                    localEdge.value(), std::move(clonedConstraint));
            }
        }

        return primitiveGraph;
    }

    [[nodiscard]] std::optional<ClusterPose> solveTrianglePrimitive(
        const ConstraintGraph& sourceGraph,
        const std::array<ConstraintGraph::NodeIdType, 3>& elements)
    {
        std::unordered_map<ConstraintGraph::NodeIdType,
            ConstraintGraph::NodeIdType>
            sourceToLocalNode;
        const auto primitiveGraph
            = buildPrimitiveSubgraph(sourceGraph, elements, sourceToLocalNode);
        if (!primitiveGraph.has_value()) {
            return std::nullopt;
        }

        ConstraintGraph solvedPrimitive = primitiveGraph.value();
        const SolveResult result = classifyAndSolve(solvedPrimitive);
        if (result.status != SolveStatus::Success) {
            return std::nullopt;
        }

        ClusterPose pose;
        for (const auto& sourceNode : elements) {
            const auto localNode = sourceToLocalNode.at(sourceNode);
            const auto solvedElement = solvedPrimitive.getElement(localNode);
            if (!solvedElement) {
                return std::nullopt;
            }

            if (solvedElement->isElementType<Point>()) {
                pose.emplace(sourceNode,
                    PointPose {
                        .position = solvedElement->getElement<Point>().position,
                    });
                continue;
            }

            if (solvedElement->isElementType<Line>()) {
                const auto& line = solvedElement->getElement<Line>();
                pose.emplace(sourceNode,
                    LinePose {
                        .p1 = line.p1,
                        .p2 = line.p2,
                    });
                continue;
            }

            return std::nullopt;
        }

        return pose;
    }

    [[nodiscard]] bool writeClusterPoseToGraph(
        const ClusterPose& pose, ConstraintGraph& targetGraph)
    {
        for (const auto& [elementId, elementPose] : pose) {
            const auto element = targetGraph.getElement(elementId);
            if (!element) {
                return false;
            }

            if (const auto point = Solvers::BottomUp::poseAsPoint(elementPose);
                point.has_value()) {
                element->updateElementPosition(point->position);
                continue;
            }

            const auto line = Solvers::BottomUp::poseAsLine(elementPose);
            if (!line.has_value()) {
                return false;
            }

            element->updateElementPosition(line->p1, line->p2);
        }

        return true;
    }

} // namespace

std::expected<void, BottomUpPlanSolveError> solveBottomUpPlans(
    const std::vector<PlanTree>& rootPlans, ConstraintGraph& graph)
{
    for (const auto& rootPlan : rootPlans) {
        const auto rootNode = rootPlan.getRoot();
        if (!rootNode.has_value()) {
            continue;
        }

        std::unordered_map<MathUtils::GeneralTreeNodeId, ClusterPose>
            solvedNodePose;

        const auto postOrderNodes = rootPlan.traversePostOrder();
        for (const auto& nodeId : postOrderNodes) {
            const auto& node = rootPlan.getValue(nodeId);

            if (node.kind == PlanNodeKind::EdgePrimitive) {
                const auto edgeInfo = std::get<EdgePrimitiveInfo>(node.info);
                const auto solved
                    = solveEdgePrimitive(graph, edgeInfo.elements);
                if (!solved.has_value()) {
                    return std::unexpected(
                        BottomUpPlanSolveError::PrimitiveSolveFailed);
                }

                solvedNodePose.emplace(nodeId, solved.value());
                continue;
            }

            if (node.kind == PlanNodeKind::TrianglePrimitive) {
                const auto triangleInfo
                    = std::get<TrianglePrimitiveInfo>(node.info);
                const auto solved
                    = solveTrianglePrimitive(graph, triangleInfo.elements);
                if (!solved.has_value()) {
                    return std::unexpected(
                        BottomUpPlanSolveError::PrimitiveSolveFailed);
                }

                solvedNodePose.emplace(nodeId, solved.value());
                continue;
            }

            if (node.kind != PlanNodeKind::Merge3) {
                return std::unexpected(BottomUpPlanSolveError::InvalidPlanNode);
            }

            const auto children = rootPlan.children(nodeId);
            if (children.size() != 3) {
                return std::unexpected(BottomUpPlanSolveError::InvalidPlanNode);
            }

            if (!solvedNodePose.contains(children[0])
                || !solvedNodePose.contains(children[1])
                || !solvedNodePose.contains(children[2])) {
                return std::unexpected(BottomUpPlanSolveError::InvalidPlanNode);
            }

            const Solvers::BottomUp::Merge3Context context {
                .sourceGraph = graph,
                .node = node,
                .children = children,
                .solvedNodePose = solvedNodePose,
            };

            if (const auto pppMergeSolution
                = Solvers::BottomUp::Merge3PppSolver::solve(context);
                pppMergeSolution.has_value()) {
                solvedNodePose.emplace(nodeId, pppMergeSolution.value());
                continue;
            }

            if (const auto pllMergeSolution
                = Solvers::BottomUp::Merge3PllSolver::solve(context);
                pllMergeSolution.has_value()) {
                solvedNodePose.emplace(nodeId, pllMergeSolution.value());
                continue;
            }

            if (const auto lppMergeSolution
                = Solvers::BottomUp::Merge3LppSolver::solve(context);
                lppMergeSolution.has_value()) {
                solvedNodePose.emplace(nodeId, lppMergeSolution.value());
                continue;
            }

            if (const auto llpMergeSolution
                = Solvers::BottomUp::Merge3LlpSolver::solve(context);
                llpMergeSolution.has_value()) {
                solvedNodePose.emplace(nodeId, llpMergeSolution.value());
                continue;
            }

            if (Solvers::BottomUp::detectUnsolvableMerge3Lll(context)) {
                return std::unexpected(
                    BottomUpPlanSolveError::MergeSolveFailed);
            }

            const auto fallback
                = Solvers::BottomUp::Merge3FallbackSolver::solve(context);
            if (!fallback.has_value()) {
                return std::unexpected(
                    BottomUpPlanSolveError::MergeSolveFailed);
            }

            solvedNodePose.emplace(nodeId, fallback.value());
        }

        if (!solvedNodePose.contains(rootNode.value())) {
            return std::unexpected(BottomUpPlanSolveError::InvalidPlanNode);
        }

        if (!writeClusterPoseToGraph(
                solvedNodePose.at(rootNode.value()), graph)) {
            return std::unexpected(BottomUpPlanSolveError::WriteBackFailed);
        }
    }

    return {};
}

} // namespace Gcs
