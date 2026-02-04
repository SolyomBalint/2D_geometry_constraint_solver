#ifndef TREE_ERRORS_HPP
#define TREE_ERRORS_HPP

namespace MathUtils {

/// @brief Errors from binary tree mutation operations.
///
/// Every MutableTree implementation uses this enum for setRoot,
/// setLeftChild, setRightChild, removeSubtree, and setValue
/// return values.
enum class TreeError {
    NodeNotFound, ///< A referenced node does not exist in the tree.
    ChildSlotOccupied, ///< The left or right child slot is already taken.
    RootAlreadyExists, ///< The tree already has a root node.
    EmptyTree, ///< Operation requires a non-empty tree.
};

} // namespace MathUtils

#endif // TREE_ERRORS_HPP
