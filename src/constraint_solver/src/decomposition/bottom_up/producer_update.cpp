#include "producer_update.hpp"

// General STD/STL headers
#include <algorithm>
#include <array>

// Custom headers
#include "edge_primitive_plan.hpp"
#include "merge_plan_builder.hpp"
#include "triangle_primitive_plan.hpp"

namespace Gcs {

namespace {

    // Build a canonical triangle object from a canonical 3-element vector.
    [[nodiscard]] std::expected<
        MathUtils::Triangle<ConstraintGraph::NodeIdType>, ProducerUpdateError>
    triangleFromElements(
        const std::vector<ConstraintGraph::NodeIdType>& outputElements)
    {
        if (outputElements.size() != 3) {
            return std::unexpected(
                ProducerUpdateError::InvalidTrianglePrimitive);
        }

        return MathUtils::Triangle<ConstraintGraph::NodeIdType> {
            .a = outputElements[0],
            .b = outputElements[1],
            .c = outputElements[2],
        };
    }

    // Build a canonical edge array from a canonical 2-element vector.
    [[nodiscard]] std::expected<std::array<ConstraintGraph::NodeIdType, 2>,
        ProducerUpdateError>
    edgeFromElements(
        const std::vector<ConstraintGraph::NodeIdType>& outputElements)
    {
        if (outputElements.size() != 2) {
            return std::unexpected(
                ProducerUpdateError::InvalidTrianglePrimitive);
        }

        return std::array<ConstraintGraph::NodeIdType, 2> {
            outputElements[0],
            outputElements[1],
        };
    }

} // namespace

std::expected<void, ProducerUpdateError> updateProducerAfterMergeThree(
    ClusterId output, std::array<ClusterId, 3> inputs,
    std::array<std::vector<ConstraintGraph::NodeIdType>, 3> inputElements,
    std::span<const ConstraintGraph::NodeIdType> outputElements,
    ProducerMap& producer)
{
    if (producer.contains(output)) {
        return std::unexpected(
            ProducerUpdateError::OutputProducerAlreadyExists);
    }

    std::array<std::size_t, 3> order { 0, 1, 2 };
    std::ranges::sort(order, [&](std::size_t lhs, std::size_t rhs) {
        return inputs[lhs] < inputs[rhs];
    });

    std::array<ClusterId, 3> sortedInputs {
        inputs[order[0]],
        inputs[order[1]],
        inputs[order[2]],
    };
    std::array<std::vector<ConstraintGraph::NodeIdType>, 3>
        sortedInputElements {
            std::move(inputElements[order[0]]),
            std::move(inputElements[order[1]]),
            std::move(inputElements[order[2]]),
        };
    const std::vector<ConstraintGraph::NodeIdType> canonicalOutputElements
        = makeCanonicalClusterElements(outputElements);

    std::array<bool, 3> hasInputProducer {
        producer.contains(sortedInputs[0]),
        producer.contains(sortedInputs[1]),
        producer.contains(sortedInputs[2]),
    };

    const bool anyInputProduced
        = hasInputProducer[0] || hasInputProducer[1] || hasInputProducer[2];
    const bool allInputsProduced
        = hasInputProducer[0] && hasInputProducer[1] && hasInputProducer[2];

    if (!anyInputProduced) {
        if (canonicalOutputElements.size() == 3) {
            const auto triangle = triangleFromElements(canonicalOutputElements);
            if (!triangle.has_value()) {
                return std::unexpected(triangle.error());
            }

            producer.emplace(
                output, makeTrianglePrimitivePlan(output, triangle.value()));
            return {};
        }

        if (canonicalOutputElements.size() == 2) {
            const auto edge = edgeFromElements(canonicalOutputElements);
            if (!edge.has_value()) {
                return std::unexpected(edge.error());
            }

            producer.emplace(
                output, makeEdgePrimitivePlan(output, edge.value()));
            return {};
        }

        return std::unexpected(ProducerUpdateError::InvalidTrianglePrimitive);
    }

    if (!allInputsProduced) {
        for (std::size_t index = 0; index < sortedInputs.size(); ++index) {
            if (hasInputProducer[index]) {
                continue;
            }

            if (sortedInputElements[index].size() == 3) {
                const auto triangle
                    = triangleFromElements(sortedInputElements[index]);
                if (!triangle.has_value()) {
                    return std::unexpected(
                        ProducerUpdateError::MissingInputProducer);
                }

                producer.emplace(sortedInputs[index],
                    makeTrianglePrimitivePlan(
                        sortedInputs[index], triangle.value()));
                hasInputProducer[index] = true;
                continue;
            }

            if (sortedInputElements[index].size() == 2) {
                const auto edge = edgeFromElements(sortedInputElements[index]);
                if (!edge.has_value()) {
                    return std::unexpected(
                        ProducerUpdateError::MissingInputProducer);
                }

                producer.emplace(sortedInputs[index],
                    makeEdgePrimitivePlan(sortedInputs[index], edge.value()));
                hasInputProducer[index] = true;
                continue;
            }

            return std::unexpected(ProducerUpdateError::MissingInputProducer);
        }
    }

    std::array<PlanTree, 3> childPlans {
        producer.at(sortedInputs[0]),
        producer.at(sortedInputs[1]),
        producer.at(sortedInputs[2]),
    };

    producer.emplace(output,
        makeMerge3Plan(output, sortedInputs, std::move(childPlans),
            canonicalOutputElements));

    producer.erase(sortedInputs[0]);
    producer.erase(sortedInputs[1]);
    producer.erase(sortedInputs[2]);

    return {};
}

} // namespace Gcs
