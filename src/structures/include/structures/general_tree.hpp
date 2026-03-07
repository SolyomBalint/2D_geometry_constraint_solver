#ifndef GENERAL_TREE_HPP
#define GENERAL_TREE_HPP

// General STD/STL headers
#include <cassert>
#include <compare>
#include <concepts>
#include <cstddef>
#include <expected>
#include <memory>
#include <optional>
#include <unordered_map>
#include <utility>
#include <vector>

// Custom headers
#include <structures/tree_errors.hpp>

namespace MathUtils {

// ============================================================
// GeneralTreeNodeId
// ============================================================

/// @brief Strongly-typed identifier for nodes in a general tree.
struct GeneralTreeNodeId {
    int value {};

    bool operator==(const GeneralTreeNodeId&) const = default;
    auto operator<=>(const GeneralTreeNodeId&) const = default;
};

} // namespace MathUtils

template <>
struct std::hash<MathUtils::GeneralTreeNodeId> {
    std::size_t operator()(
        const MathUtils::GeneralTreeNodeId& id) const noexcept
    {
        return std::hash<int> {}(id.value);
    }
};

namespace MathUtils {

// ============================================================
// GeneralTree
// ============================================================

/**
 * @brief A general-purpose rooted tree storing values of type @p T.
 *
 * Each node can have an arbitrary number of children. Child ordering is
 * insertion-order and is used by traversal operations.
 *
 * The tree owns all nodes via @c std::unique_ptr while still providing value
 * semantics through explicit deep-copy operations.
 *
 * @tparam T Value type stored in each node.
 */
template <typename T>
class GeneralTree {
public:
    using NodeIdType = GeneralTreeNodeId;
    using ValueType = T;

    GeneralTree() = default;

    /**
     * @brief Construct a single-node tree with @p rootValue as root payload.
     */
    explicit GeneralTree(T rootValue)
    {
        setRootUnchecked(std::move(rootValue));
    }

    GeneralTree(const GeneralTree& other)
        : m_nextNodeId { other.m_nextNodeId }
        , m_size { other.m_size }
        , m_root { cloneSubtree(other.m_root.get()) }
    {
        rebuildNodeIndex();
    }

    GeneralTree& operator=(const GeneralTree& other)
    {
        if (this == &other)
            return *this;

        GeneralTree copy { other };
        swap(copy);
        return *this;
    }

    GeneralTree(GeneralTree&&) noexcept = default;
    GeneralTree& operator=(GeneralTree&&) noexcept = default;
    ~GeneralTree() = default;

    void swap(GeneralTree& other) noexcept
    {
        std::swap(m_nextNodeId, other.m_nextNodeId);
        std::swap(m_size, other.m_size);
        std::swap(m_root, other.m_root);
        std::swap(m_nodeIndex, other.m_nodeIndex);
    }

    // --------------------------------------------------------
    // Read operations
    // --------------------------------------------------------

    /**
     * @brief Returns the root node identifier, if the tree is non-empty.
     */
    [[nodiscard]] std::optional<NodeIdType> getRoot() const
    {
        if (!m_root)
            return std::nullopt;
        return m_root->id;
    }

    /**
     * @brief Tests whether a node exists in this tree.
     */
    [[nodiscard]] bool hasNode(NodeIdType id) const
    {
        return m_nodeIndex.contains(id);
    }

    /**
     * @brief Tests whether the tree is empty.
     */
    [[nodiscard]] bool isEmpty() const { return m_root == nullptr; }

    /**
     * @brief Returns the total number of nodes in the tree.
     */
    [[nodiscard]] std::size_t size() const { return m_size; }

    /**
     * @brief Returns the number of leaf nodes in the tree.
     */
    [[nodiscard]] std::size_t leafCount() const
    {
        if (!m_root)
            return 0;
        return countLeaves(m_root.get());
    }

    /**
     * @brief Returns the value stored at a node.
     * @pre @p id must be a valid node in this tree.
     */
    [[nodiscard]] const T& getValue(NodeIdType id) const
    {
        return findNode(id).value;
    }

    /**
     * @brief Returns mutable access to the value stored at a node.
     * @pre @p id must be a valid node in this tree.
     */
    [[nodiscard]] T& getValue(NodeIdType id) { return findNode(id).value; }

    /**
     * @brief Tests whether a node is a leaf (has no children).
     * @pre @p id must be a valid node in this tree.
     */
    [[nodiscard]] bool isLeaf(NodeIdType id) const
    {
        const Node& node = findNode(id);
        return node.children.empty();
    }

    /**
     * @brief Returns child node IDs of @p parentId in insertion order.
     * @pre @p parentId must be a valid node in this tree.
     */
    [[nodiscard]] std::vector<NodeIdType> children(NodeIdType parentId) const
    {
        const Node& parent = findNode(parentId);
        std::vector<NodeIdType> childIds;
        childIds.reserve(parent.children.size());
        for (const auto& child : parent.children)
            childIds.push_back(child->id);
        return childIds;
    }

    /**
     * @brief Returns child node IDs of @p parentId in insertion order.
     * @pre @p parentId must be a valid node in this tree.
     */
    [[nodiscard]] std::vector<NodeIdType> children(NodeIdType parentId)
    {
        const Node& parent = findNode(parentId);
        std::vector<NodeIdType> childIds;
        childIds.reserve(parent.children.size());
        for (const auto& child : parent.children)
            childIds.push_back(child->id);
        return childIds;
    }

    // --------------------------------------------------------
    // Mutation operations
    // --------------------------------------------------------

    /**
     * @brief Creates the root node if the tree is empty.
     * @return The new root ID, or @c TreeError::RootAlreadyExists.
     */
    std::expected<NodeIdType, TreeError> setRoot(T value)
    {
        if (m_root)
            return std::unexpected(TreeError::RootAlreadyExists);

        return setRootUnchecked(std::move(value));
    }

    /**
     * @brief Appends a new child to @p parentId.
     * @return The new child's ID, or @c TreeError::NodeNotFound.
     */
    std::expected<NodeIdType, TreeError> addChild(NodeIdType parentId, T value)
    {
        auto it = m_nodeIndex.find(parentId);
        if (it == m_nodeIndex.end())
            return std::unexpected(TreeError::NodeNotFound);

        Node* parent = it->second;
        NodeIdType childId { m_nextNodeId++ };
        auto child = std::make_unique<Node>(Node {
            .id = childId,
            .value = std::move(value),
            .children = {},
        });

        Node* childPtr = child.get();
        parent->children.push_back(std::move(child));
        m_nodeIndex.emplace(childId, childPtr);
        ++m_size;
        return childId;
    }

    // --------------------------------------------------------
    // Traversal operations
    // --------------------------------------------------------

    /**
     * @brief Returns node IDs in pre-order sequence (root, children...).
     */
    [[nodiscard]] std::vector<NodeIdType> traversePreOrder() const
    {
        std::vector<NodeIdType> result;
        result.reserve(m_size);
        if (m_root)
            preOrder(m_root.get(), result);
        return result;
    }

    /**
     * @brief Returns node IDs in post-order sequence (children..., root).
     */
    [[nodiscard]] std::vector<NodeIdType> traversePostOrder() const
    {
        std::vector<NodeIdType> result;
        result.reserve(m_size);
        if (m_root)
            postOrder(m_root.get(), result);
        return result;
    }

private:
    struct Node {
        NodeIdType id;
        T value;
        std::vector<std::unique_ptr<Node>> children;
    };

    [[nodiscard]] NodeIdType setRootUnchecked(T value)
    {
        NodeIdType rootId { m_nextNodeId++ };
        m_root = std::make_unique<Node>(Node {
            .id = rootId,
            .value = std::move(value),
            .children = {},
        });

        m_nodeIndex.clear();
        m_nodeIndex.emplace(rootId, m_root.get());
        m_size = 1;
        return rootId;
    }

    [[nodiscard]] Node& findNode(NodeIdType id)
    {
        auto it = m_nodeIndex.find(id);
        assert(
            it != m_nodeIndex.end() && "findNode: invalid GeneralTreeNodeId");
        return *it->second;
    }

    [[nodiscard]] const Node& findNode(NodeIdType id) const
    {
        auto it = m_nodeIndex.find(id);
        assert(
            it != m_nodeIndex.end() && "findNode: invalid GeneralTreeNodeId");
        return *it->second;
    }

    [[nodiscard]] std::unique_ptr<Node> cloneSubtree(const Node* source) const
    {
        if (!source)
            return nullptr;

        auto cloned = std::make_unique<Node>(Node {
            .id = source->id,
            .value = source->value,
            .children = {},
        });
        cloned->children.reserve(source->children.size());

        for (const auto& child : source->children)
            cloned->children.push_back(cloneSubtree(child.get()));

        return cloned;
    }

    void rebuildNodeIndex()
    {
        m_nodeIndex.clear();
        if (!m_root)
            return;

        indexSubtree(m_root.get());
    }

    void indexSubtree(Node* node)
    {
        assert(node != nullptr && "indexSubtree: node must not be null");
        m_nodeIndex.emplace(node->id, node);
        for (const auto& child : node->children)
            indexSubtree(child.get());
    }

    [[nodiscard]] std::size_t countLeaves(const Node* node) const
    {
        assert(node != nullptr && "countLeaves: node must not be null");
        if (node->children.empty())
            return 1;

        std::size_t count = 0;
        for (const auto& child : node->children)
            count += countLeaves(child.get());
        return count;
    }

    void preOrder(const Node* node, std::vector<NodeIdType>& result) const
    {
        assert(node != nullptr && "preOrder: node must not be null");
        result.push_back(node->id);
        for (const auto& child : node->children)
            preOrder(child.get(), result);
    }

    void postOrder(const Node* node, std::vector<NodeIdType>& result) const
    {
        assert(node != nullptr && "postOrder: node must not be null");
        for (const auto& child : node->children)
            postOrder(child.get(), result);
        result.push_back(node->id);
    }

    int m_nextNodeId { 0 };
    std::size_t m_size { 0 };
    std::unique_ptr<Node> m_root;
    std::unordered_map<NodeIdType, Node*> m_nodeIndex;
};

// ============================================================
// Concept verification
// ============================================================

static_assert(std::copy_constructible<GeneralTree<int>>,
    "GeneralTree<int> must support deep copy semantics");
static_assert(std::default_initializable<GeneralTree<int>>,
    "GeneralTree<int> must be default constructible");

} // namespace MathUtils

#endif // GENERAL_TREE_HPP
