#ifndef CONSTRAINT_GRAPH_CONVERTER_HPP
#define CONSTRAINT_GRAPH_CONVERTER_HPP

#include "simple_constraint_graph.hpp"
#include "../constraint_solver/geometric_constraint_system.hpp"

namespace ConstraintGraphConverter {

/**
 * @brief Converts a SimpleConstraintGraph to a full ConstraintGraph
 * @param simpleGraph The simple constraint graph from Python
 * @return A ConstraintGraph ready for use with the geometric constraint system
 */
Gcs::ConstraintGraph convertToConstraintGraph(
    const SimpleCG::SimpleConstraintGraph& simpleGraph);

/**
 * @brief Helper function to convert SimpleElement to Element
 * @param simpleElement The simple element to convert
 * @return A shared pointer to the converted Element
 */
std::shared_ptr<Gcs::Element> convertElement(
    const SimpleCG::SimpleElement& simpleElement);

/**
 * @brief Helper function to convert SimpleConstraint to Constraint
 * @param simpleConstraint The simple constraint to convert
 * @return A shared pointer to the converted Constraint
 */
std::shared_ptr<Gcs::Constraint> convertConstraint(
    const SimpleCG::SimpleConstraint& simpleConstraint);

/**
 * @brief Converts a ConstraintGraph back to a SimpleConstraintGraph
 * @param constraintGraph The full constraint graph to convert
 * @return A SimpleConstraintGraph for use with Python
 */
SimpleCG::SimpleConstraintGraph convertFromConstraintGraph(
    const Gcs::ConstraintGraph& constraintGraph);

/**
 * @brief Helper function to convert Element back to SimpleElement
 * @param element The element to convert
 * @return A SimpleElement
 */
SimpleCG::SimpleElement convertElementBack(
    const std::shared_ptr<Gcs::Element>& element);

/**
 * @brief Helper function to convert Constraint back to SimpleConstraint
 * @param constraint The constraint to convert
 * @return A SimpleConstraint
 */
SimpleCG::SimpleConstraint convertConstraintBack(
    const std::shared_ptr<Gcs::Constraint>& constraint);

} // namespace ConstraintGraphConverter

#endif // CONSTRAINT_GRAPH_CONVERTER_HPP
