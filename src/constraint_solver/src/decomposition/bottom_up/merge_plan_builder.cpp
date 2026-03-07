#include "merge_plan_builder.hpp"

// General STD/STL headers
#include <algorithm>
#include <array>
#include <cassert>
#include <cstddef>

// Custom headers
#include "cluster_types.hpp"

namespace Gcs {

namespace {

    // Recursively clone one source subtree under a destination parent node.
    void appendSubtree(MathUtils::GeneralTree<PlanNode>& destination,
        MathUtils::GeneralTree<PlanNode>::NodeIdType destinationParent,
        const MathUtils::GeneralTree<PlanNode>& source,
        MathUtils::GeneralTree<PlanNode>::NodeIdType sourceNode)
    {
        const auto addedNode = destination.addChild(
            destinationParent, source.getValue(sourceNode));
        assert(addedNode.has_value() && "Destination parent node must exist");

        for (const auto& sourceChild : source.children(sourceNode)) {
            appendSubtree(destination, addedNode.value(), source, sourceChild);
        }
    }

    // Attach one plan tree as a child subtree of a destination parent node.
    void appendChildPlan(MathUtils::GeneralTree<PlanNode>& destination,
        MathUtils::GeneralTree<PlanNode>::NodeIdType destinationParent,
        const MathUtils::GeneralTree<PlanNode>& childPlan)
    {
        const auto childRoot = childPlan.getRoot();
        assert(childRoot.has_value() && "Child plan must not be empty");
        appendSubtree(
            destination, destinationParent, childPlan, childRoot.value());
    }

} // namespace

MathUtils::GeneralTree<PlanNode> makeMerge3Plan(ClusterId output,
    std::array<ClusterId, 3> inputs,
    std::array<MathUtils::GeneralTree<PlanNode>, 3> childPlans,
    std::span<const ConstraintGraph::NodeIdType> outputElements)
{
    std::array<std::size_t, 3> order { 0, 1, 2 };
    std::ranges::sort(order, [&](std::size_t lhs, std::size_t rhs) {
        return inputs[lhs] < inputs[rhs];
    });

    std::array<ClusterId, 3> sortedInputs {
        inputs[order[0]],
        inputs[order[1]],
        inputs[order[2]],
    };
    std::array<MathUtils::GeneralTree<PlanNode>, 3> sortedChildPlans {
        std::move(childPlans[order[0]]),
        std::move(childPlans[order[1]]),
        std::move(childPlans[order[2]]),
    };

    const PlanNode mergeRoot {
        .kind = PlanNodeKind::Merge3,
        .info = Merge3Info {
            .output = output,
            .inputs = sortedInputs,
            .outputElements = makeCanonicalClusterElements(outputElements),
        },
    };

    MathUtils::GeneralTree<PlanNode> tree { mergeRoot };
    const auto rootId = tree.getRoot();
    assert(rootId.has_value() && "Merge plan root must exist");

    appendChildPlan(tree, rootId.value(), sortedChildPlans[0]);
    appendChildPlan(tree, rootId.value(), sortedChildPlans[1]);
    appendChildPlan(tree, rootId.value(), sortedChildPlans[2]);

    return tree;
}

} // namespace Gcs
