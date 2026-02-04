#ifndef BINARY_TREE_HPP
#define BINARY_TREE_HPP

// General STD/STL headers
#include <algorithm>
#include <cassert>
#include <compare>
#include <concepts>
#include <cstddef>
#include <expected>
#include <functional>
#include <optional>
#include <queue>
#include <ranges>
#include <unordered_map>
#include <vector>

// Custom headers
#include <structures/tree_errors.hpp>

namespace MathUtils {

// ============================================================
// TreeNodeId
// ============================================================

/// @brief Strongly-typed identifier for nodes in a binary tree.
struct TreeNodeId {
    int value {};

    bool operator==(const TreeNodeId&) const = default;
    auto operator<=>(const TreeNodeId&) const = default;
};

} // namespace MathUtils

template <>
struct std::hash<MathUtils::TreeNodeId> {
    std::size_t operator()(const MathUtils::TreeNodeId& id) const noexcept
    {
        return std::hash<int> {}(id.value);
    }
};

namespace MathUtils {

// ============================================================
// Tree concepts
// ============================================================

/**
 * @brief Read-only interface for a binary tree.
 *
 * Any type satisfying TreeBase provides const access to
 * tree structure (root, parent, children) and node values,
 * as well as size and height queries.
 */
template <typename T>
concept TreeBase = requires(const T tree, typename T::NodeIdType id) {
    typename T::NodeIdType;
    typename T::ValueType;

    { tree.getRoot() } -> std::same_as<std::optional<typename T::NodeIdType>>;
    { tree.getValue(id) } -> std::same_as<const typename T::ValueType&>;
    {
        tree.getLeftChild(id)
    } -> std::same_as<std::optional<typename T::NodeIdType>>;
    {
        tree.getRightChild(id)
    } -> std::same_as<std::optional<typename T::NodeIdType>>;
    {
        tree.getParent(id)
    } -> std::same_as<std::optional<typename T::NodeIdType>>;
    { tree.hasNode(id) } -> std::same_as<bool>;
    { tree.nodeCount() } -> std::same_as<std::size_t>;
    { tree.isEmpty() } -> std::same_as<bool>;
    { tree.height() } -> std::same_as<std::size_t>;
};

/**
 * @brief Mutable interface for a binary tree.
 *
 * Extends TreeBase with operations that modify tree structure
 * and node values. All mutating operations return
 * @c std::expected to signal errors via @c TreeError.
 */
template <typename T>
concept MutableTree = TreeBase<T>
    && requires(
        T tree, typename T::NodeIdType id, typename T::ValueType value) {
           {
               tree.setRoot(value)
           } -> std::same_as<std::expected<typename T::NodeIdType, TreeError>>;
           {
               tree.setLeftChild(id, value)
           } -> std::same_as<std::expected<typename T::NodeIdType, TreeError>>;
           {
               tree.setRightChild(id, value)
           } -> std::same_as<std::expected<typename T::NodeIdType, TreeError>>;
           {
               tree.removeSubtree(id)
           } -> std::same_as<std::expected<void, TreeError>>;
           {
               tree.setValue(id, value)
           } -> std::same_as<std::expected<void, TreeError>>;
       };

// ============================================================
// BinaryTree
// ============================================================

/**
 * @brief A general-purpose binary tree storing values of type
 *        @p T at each node.
 *
 * Nodes are identified by TreeNodeId and placed explicitly as
 * left or right children. No ordering invariant is maintained.
 *
 * @tparam T The value type stored in each node. Must be
 *           move-constructible.
 */
template <typename T>
class BinaryTree {
public:
    using NodeIdType = TreeNodeId;
    using ValueType = T;

    BinaryTree() = default;

    // --------------------------------------------------------
    // Read operations
    // --------------------------------------------------------

    /**
     * @brief Returns the root node identifier, if the tree is
     *        non-empty.
     * @return The root node ID, or @c std::nullopt for an
     *         empty tree.
     */
    [[nodiscard]] std::optional<TreeNodeId> getRoot() const { return m_root; }

    /**
     * @brief Returns the value stored at a node.
     * @param id The node to query.
     * @return A const reference to the stored value.
     * @pre @p id must be a valid node in this tree.
     */
    [[nodiscard]] const T& getValue(TreeNodeId id) const
    {
        auto it = m_nodes.find(id);
        assert(it != m_nodes.end() && "getValue: invalid TreeNodeId");
        return it->second.value;
    }

    /**
     * @brief Returns the left child of a node.
     * @param id The parent node to query.
     * @return The left child's ID, or @c std::nullopt if none.
     * @pre @p id must be a valid node in this tree.
     */
    [[nodiscard]] std::optional<TreeNodeId> getLeftChild(TreeNodeId id) const
    {
        auto it = m_nodes.find(id);
        assert(it != m_nodes.end() && "getLeftChild: invalid TreeNodeId");
        return it->second.leftChild;
    }

    /**
     * @brief Returns the right child of a node.
     * @param id The parent node to query.
     * @return The right child's ID, or @c std::nullopt if none.
     * @pre @p id must be a valid node in this tree.
     */
    [[nodiscard]] std::optional<TreeNodeId> getRightChild(TreeNodeId id) const
    {
        auto it = m_nodes.find(id);
        assert(it != m_nodes.end() && "getRightChild: invalid TreeNodeId");
        return it->second.rightChild;
    }

    /**
     * @brief Returns the parent of a node.
     * @param id The node to query.
     * @return The parent's ID, or @c std::nullopt if @p id is
     *         the root.
     * @pre @p id must be a valid node in this tree.
     */
    [[nodiscard]] std::optional<TreeNodeId> getParent(TreeNodeId id) const
    {
        auto it = m_nodes.find(id);
        assert(it != m_nodes.end() && "getParent: invalid TreeNodeId");
        return it->second.parent;
    }

    /**
     * @brief Tests whether a node exists in this tree.
     * @param id The node ID to test.
     * @return @c true if the node exists, @c false otherwise.
     */
    [[nodiscard]] bool hasNode(TreeNodeId id) const
    {
        return m_nodes.contains(id);
    }

    /**
     * @brief Returns the number of nodes in the tree.
     * @return The node count.
     */
    [[nodiscard]] std::size_t nodeCount() const { return m_nodes.size(); }

    /**
     * @brief Tests whether the tree is empty.
     * @return @c true if the tree has no nodes.
     */
    [[nodiscard]] bool isEmpty() const { return m_nodes.empty(); }

    /**
     * @brief Computes the height of the tree.
     *
     * The height of an empty tree is 0. A single-node tree has
     * height 1. Otherwise it is 1 + max(left height, right
     * height).
     *
     * @return The height of the tree.
     */
    [[nodiscard]] std::size_t height() const
    {
        if (!m_root)
            return 0;
        return computeHeight(*m_root);
    }

    /**
     * @brief Tests whether a node is a leaf (has no children).
     * @param id The node to test.
     * @return @c true if the node has no children.
     * @pre @p id must be a valid node in this tree.
     */
    [[nodiscard]] bool isLeaf(TreeNodeId id) const
    {
        auto it = m_nodes.find(id);
        assert(it != m_nodes.end() && "isLeaf: invalid TreeNodeId");
        return !it->second.leftChild && !it->second.rightChild;
    }

    /**
     * @brief Returns the IDs of all leaf nodes (nodes with no
     *        children).
     * @return A vector of leaf node IDs. Empty if the tree is
     *         empty.
     */
    [[nodiscard]] std::vector<TreeNodeId> getLeaves() const
    {
        std::vector<TreeNodeId> leaves;
        for (const auto& [id, node] : m_nodes) {
            if (!node.leftChild && !node.rightChild)
                leaves.push_back(id);
        }
        return leaves;
    }

    /**
     * @brief Returns copies of the values stored at all leaf
     *        nodes.
     *
     * Collects the value of every node that has no children.
     *
     * @return A vector of copied values from leaf nodes. Empty
     *         if the tree is empty.
     */
    [[nodiscard]] std::vector<T> getLeafValues() const
    {
        std::vector<T> values;
        for (const auto& [id, node] : m_nodes) {
            if (!node.leftChild && !node.rightChild)
                values.push_back(node.value);
        }
        return values;
    }

    // --------------------------------------------------------
    // Factory
    // --------------------------------------------------------

    /**
     * @brief Constructs a tree from a root value and two
     *        optional child subtrees.
     *
     * Moves all nodes out of @p left and @p right, re-keying
     * their IDs into the new tree. Both source trees are left
     * in a valid but empty state after the call.
     *
     * @param rootValue The value to store at the new root.
     * @param left  The left subtree (may be empty).
     * @param right The right subtree (may be empty).
     * @return A new tree with @p rootValue at the root and
     *         the two subtrees attached.
     */
    static BinaryTree make(T rootValue, BinaryTree left, BinaryTree right)
    {
        BinaryTree tree;
        TreeNodeId rootId { tree.m_nextNodeId++ };
        tree.m_nodes[rootId] = Node {
            .value = std::move(rootValue),
            .parent = std::nullopt,
            .leftChild = std::nullopt,
            .rightChild = std::nullopt,
        };
        tree.m_root = rootId;

        if (left.m_root) {
            TreeNodeId leftRootId = tree.graftSubtree(left, *left.m_root);
            tree.m_nodes[rootId].leftChild = leftRootId;
            tree.m_nodes[leftRootId].parent = rootId;
        }

        if (right.m_root) {
            TreeNodeId rightRootId = tree.graftSubtree(right, *right.m_root);
            tree.m_nodes[rootId].rightChild = rightRootId;
            tree.m_nodes[rightRootId].parent = rootId;
        }

        return tree;
    }

    // --------------------------------------------------------
    // Mutation operations
    // --------------------------------------------------------

    /**
     * @brief Creates the root node with the given value.
     * @param value The value to store at the root.
     * @return The new root's ID, or @c TreeError::RootAlreadyExists
     *         if a root already exists.
     */
    std::expected<TreeNodeId, TreeError> setRoot(T value)
    {
        if (m_root)
            return std::unexpected(TreeError::RootAlreadyExists);

        TreeNodeId id { m_nextNodeId++ };
        m_nodes[id] = Node {
            .value = std::move(value),
            .parent = std::nullopt,
            .leftChild = std::nullopt,
            .rightChild = std::nullopt,
        };
        m_root = id;
        return id;
    }

    /**
     * @brief Creates a left child for the given parent.
     * @param parentId The parent node.
     * @param value The value to store at the new child.
     * @return The new child's ID, or a @c TreeError if
     *         @p parentId is invalid or the left slot is
     *         occupied.
     */
    std::expected<TreeNodeId, TreeError> setLeftChild(
        TreeNodeId parentId, T value)
    {
        auto it = m_nodes.find(parentId);
        if (it == m_nodes.end())
            return std::unexpected(TreeError::NodeNotFound);
        if (it->second.leftChild)
            return std::unexpected(TreeError::ChildSlotOccupied);

        TreeNodeId id { m_nextNodeId++ };
        m_nodes[id] = Node {
            .value = std::move(value),
            .parent = parentId,
            .leftChild = std::nullopt,
            .rightChild = std::nullopt,
        };
        it->second.leftChild = id;
        return id;
    }

    /**
     * @brief Creates a right child for the given parent.
     * @param parentId The parent node.
     * @param value The value to store at the new child.
     * @return The new child's ID, or a @c TreeError if
     *         @p parentId is invalid or the right slot is
     *         occupied.
     */
    std::expected<TreeNodeId, TreeError> setRightChild(
        TreeNodeId parentId, T value)
    {
        auto it = m_nodes.find(parentId);
        if (it == m_nodes.end())
            return std::unexpected(TreeError::NodeNotFound);
        if (it->second.rightChild)
            return std::unexpected(TreeError::ChildSlotOccupied);

        TreeNodeId id { m_nextNodeId++ };
        m_nodes[id] = Node {
            .value = std::move(value),
            .parent = parentId,
            .leftChild = std::nullopt,
            .rightChild = std::nullopt,
        };
        it->second.rightChild = id;
        return id;
    }

    /**
     * @brief Removes a node and all of its descendants.
     *
     * If @p id is the root, the entire tree is cleared. If
     * @p id has a parent, the corresponding child link in the
     * parent is reset to @c std::nullopt.
     *
     * @param id The root of the subtree to remove.
     * @return Void on success, or @c TreeError::NodeNotFound.
     */
    std::expected<void, TreeError> removeSubtree(TreeNodeId id)
    {
        auto it = m_nodes.find(id);
        if (it == m_nodes.end())
            return std::unexpected(TreeError::NodeNotFound);

        // Unlink from parent
        if (it->second.parent) {
            auto parentIt = m_nodes.find(*it->second.parent);
            assert(parentIt != m_nodes.end()
                && "removeSubtree: parent invariant violated");

            if (parentIt->second.leftChild == id)
                parentIt->second.leftChild = std::nullopt;
            else if (parentIt->second.rightChild == id)
                parentIt->second.rightChild = std::nullopt;
        }

        // Clear root if removing root
        if (m_root == id)
            m_root = std::nullopt;

        // Recursively remove all descendants
        removeSubtreeNodes(id);
        return {};
    }

    /**
     * @brief Replaces the value stored at a node.
     * @param id The node whose value to replace.
     * @param value The new value.
     * @return Void on success, or @c TreeError::NodeNotFound.
     */
    std::expected<void, TreeError> setValue(TreeNodeId id, T value)
    {
        auto it = m_nodes.find(id);
        if (it == m_nodes.end())
            return std::unexpected(TreeError::NodeNotFound);
        it->second.value = std::move(value);
        return {};
    }

    // --------------------------------------------------------
    // Traversal operations
    // --------------------------------------------------------

    /**
     * @brief Returns nodes in in-order sequence (left, root,
     *        right).
     * @return A vector of node IDs in in-order.
     */
    [[nodiscard]] std::vector<TreeNodeId> traverseInOrder() const
    {
        std::vector<TreeNodeId> result;
        if (m_root)
            inOrder(*m_root, result);
        return result;
    }

    /**
     * @brief Returns nodes in pre-order sequence (root, left,
     *        right).
     * @return A vector of node IDs in pre-order.
     */
    [[nodiscard]] std::vector<TreeNodeId> traversePreOrder() const
    {
        std::vector<TreeNodeId> result;
        if (m_root)
            preOrder(*m_root, result);
        return result;
    }

    /**
     * @brief Returns nodes in post-order sequence (left, right,
     *        root).
     * @return A vector of node IDs in post-order.
     */
    [[nodiscard]] std::vector<TreeNodeId> traversePostOrder() const
    {
        std::vector<TreeNodeId> result;
        if (m_root)
            postOrder(*m_root, result);
        return result;
    }

    /**
     * @brief Returns leaf node IDs in right-first post-order.
     *
     * Performs a post-order walk that visits the right subtree
     * before the left subtree and only collects leaf nodes
     * (nodes with no children). This yields an ordering where
     * the rightmost leaf appears first.
     *
     * @return A vector of leaf node IDs in right-first
     *         post-order. Empty if the tree is empty.
     */
    [[nodiscard]] std::vector<TreeNodeId> getLeafIdsPostOrder() const
    {
        std::vector<TreeNodeId> result;
        if (m_root)
            postOrderRightFirstLeaves(*m_root, result);
        return result;
    }

    /**
     * @brief Returns copies of leaf values in right-first
     *        post-order.
     *
     * Equivalent to calling @c getLeafIdsPostOrder and then
     * fetching the value at each returned ID. The rightmost
     * leaf's value appears first.
     *
     * @return A vector of copied values from leaf nodes in
     *         right-first post-order. Empty if the tree is
     *         empty.
     */
    [[nodiscard]] std::vector<T> getLeafValuesPostOrder() const
    {
        auto ids = getLeafIdsPostOrder();
        std::vector<T> values;
        values.reserve(ids.size());
        for (auto id : ids)
            values.push_back(getValue(id));
        return values;
    }

    /**
     * @brief Returns nodes in level-order (breadth-first)
     *        sequence.
     * @return A vector of node IDs in level-order.
     */
    [[nodiscard]] std::vector<TreeNodeId> traverseLevelOrder() const
    {
        std::vector<TreeNodeId> result;
        if (!m_root)
            return result;

        std::queue<TreeNodeId> frontier;
        frontier.push(*m_root);

        while (!frontier.empty()) {
            TreeNodeId current = frontier.front();
            frontier.pop();
            result.push_back(current);

            auto it = m_nodes.find(current);
            assert(it != m_nodes.end()
                && "traverseLevelOrder: node invariant violated");

            if (it->second.leftChild)
                frontier.push(*it->second.leftChild);
            if (it->second.rightChild)
                frontier.push(*it->second.rightChild);
        }
        return result;
    }

    // --------------------------------------------------------
    // Search
    // --------------------------------------------------------

    /**
     * @brief Finds the first node storing a given value.
     *
     * Performs a linear scan over all nodes (no ordering
     * invariant). Returns the first match found, or
     * @c std::nullopt if no node contains the value.
     *
     * @param value The value to search for.
     * @return The matching node's ID, or @c std::nullopt.
     */
    [[nodiscard]] std::optional<TreeNodeId> find(const T& value) const
    {
        for (const auto& [id, node] : m_nodes) {
            if (node.value == value)
                return id;
        }
        return std::nullopt;
    }

private:
    // --------------------------------------------------------
    // Internal node representation
    // --------------------------------------------------------

    struct Node {
        T value;
        std::optional<TreeNodeId> parent;
        std::optional<TreeNodeId> leftChild;
        std::optional<TreeNodeId> rightChild;
    };

    // --------------------------------------------------------
    // Private helpers
    // --------------------------------------------------------

    /**
     * @brief Moves an entire subtree from @p source into this
     *        tree, assigning fresh IDs.
     *
     * Recursively walks @p source starting at @p srcId and
     * inserts each node with a new ID from this tree's counter.
     * Parent links within the grafted subtree are set correctly;
     * the caller is responsible for setting the parent link of
     * the returned root and the child link in the receiving
     * parent.
     *
     * @param source The tree to move nodes from.
     * @param srcId  The root of the subtree in @p source.
     * @return The new ID of the grafted subtree root in this
     *         tree.
     */
    TreeNodeId graftSubtree(BinaryTree& source, TreeNodeId srcId)
    {
        auto srcIt = source.m_nodes.find(srcId);
        assert(srcIt != source.m_nodes.end() && "graftSubtree: invalid srcId");

        TreeNodeId newId { m_nextNodeId++ };
        m_nodes[newId] = Node {
            .value = std::move(srcIt->second.value),
            .parent = std::nullopt,
            .leftChild = std::nullopt,
            .rightChild = std::nullopt,
        };

        if (srcIt->second.leftChild) {
            TreeNodeId leftId = graftSubtree(source, *srcIt->second.leftChild);
            m_nodes[newId].leftChild = leftId;
            m_nodes[leftId].parent = newId;
        }

        if (srcIt->second.rightChild) {
            TreeNodeId rightId
                = graftSubtree(source, *srcIt->second.rightChild);
            m_nodes[newId].rightChild = rightId;
            m_nodes[rightId].parent = newId;
        }

        source.m_nodes.erase(srcIt);
        return newId;
    }

    /**
     * @brief Recursively computes the height of the subtree
     *        rooted at @p id.
     */
    std::size_t computeHeight(TreeNodeId id) const
    {
        auto it = m_nodes.find(id);
        assert(it != m_nodes.end() && "computeHeight: invalid TreeNodeId");

        std::size_t leftHeight = 0;
        std::size_t rightHeight = 0;

        if (it->second.leftChild)
            leftHeight = computeHeight(*it->second.leftChild);
        if (it->second.rightChild)
            rightHeight = computeHeight(*it->second.rightChild);

        return 1 + std::max(leftHeight, rightHeight);
    }

    /**
     * @brief Recursively removes a node and all descendants
     *        from @c m_nodes.
     */
    void removeSubtreeNodes(TreeNodeId id)
    {
        auto it = m_nodes.find(id);
        if (it == m_nodes.end())
            return;

        if (it->second.leftChild)
            removeSubtreeNodes(*it->second.leftChild);
        if (it->second.rightChild)
            removeSubtreeNodes(*it->second.rightChild);

        m_nodes.erase(it);
    }

    /**
     * @brief In-order traversal helper (left, root, right).
     */
    void inOrder(TreeNodeId id, std::vector<TreeNodeId>& result) const
    {
        auto it = m_nodes.find(id);
        assert(it != m_nodes.end() && "inOrder: invalid TreeNodeId");

        if (it->second.leftChild)
            inOrder(*it->second.leftChild, result);
        result.push_back(id);
        if (it->second.rightChild)
            inOrder(*it->second.rightChild, result);
    }

    /**
     * @brief Pre-order traversal helper (root, left, right).
     */
    void preOrder(TreeNodeId id, std::vector<TreeNodeId>& result) const
    {
        auto it = m_nodes.find(id);
        assert(it != m_nodes.end() && "preOrder: invalid TreeNodeId");

        result.push_back(id);
        if (it->second.leftChild)
            preOrder(*it->second.leftChild, result);
        if (it->second.rightChild)
            preOrder(*it->second.rightChild, result);
    }

    /**
     * @brief Post-order traversal helper (left, right, root).
     */
    void postOrder(TreeNodeId id, std::vector<TreeNodeId>& result) const
    {
        auto it = m_nodes.find(id);
        assert(it != m_nodes.end() && "postOrder: invalid TreeNodeId");

        if (it->second.leftChild)
            postOrder(*it->second.leftChild, result);
        if (it->second.rightChild)
            postOrder(*it->second.rightChild, result);
        result.push_back(id);
    }

    /**
     * @brief Right-first post-order helper collecting only leaf
     *        nodes (right, left, skip-internal).
     */
    void postOrderRightFirstLeaves(
        TreeNodeId id, std::vector<TreeNodeId>& result) const
    {
        auto it = m_nodes.find(id);
        assert(it != m_nodes.end()
            && "postOrderRightFirstLeaves: invalid TreeNodeId");

        bool leaf = !it->second.leftChild && !it->second.rightChild;
        if (leaf) {
            result.push_back(id);
            return;
        }

        if (it->second.rightChild)
            postOrderRightFirstLeaves(*it->second.rightChild, result);
        if (it->second.leftChild)
            postOrderRightFirstLeaves(*it->second.leftChild, result);
    }

    // --------------------------------------------------------
    // Data members
    // --------------------------------------------------------

    int m_nextNodeId { 0 };
    std::optional<TreeNodeId> m_root;
    std::unordered_map<TreeNodeId, Node> m_nodes;
};

// ============================================================
// Concept verification
// ============================================================

static_assert(
    TreeBase<BinaryTree<int>>, "BinaryTree<int> must satisfy TreeBase");
static_assert(
    MutableTree<BinaryTree<int>>, "BinaryTree<int> must satisfy MutableTree");

} // namespace MathUtils

#endif // BINARY_TREE_HPP
