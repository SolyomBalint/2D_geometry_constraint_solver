#ifndef PROPERTY_MAP_HPP
#define PROPERTY_MAP_HPP

// Custom headers
#include <structures/graph_errors.hpp>
#include <structures/simple_graph.hpp>

// General STD/STL headers
#include <cstddef>
#include <expected>
#include <flat_map>
#include <functional>
#include <ranges>
#include <utility>

namespace MathUtils {

/**
 * @brief Associative property storage keyed by graph element IDs.
 *
 * Maps graph element IDs (nodes or edges) to user-defined values using
 * a configurable backing container. Not graph-aware: does not track
 * insertions or removals automatically.
 *
 * @tparam KeyId     The key type (must satisfy @c NodeIdentity or
 *                   @c EdgeIdentity).
 * @tparam Value     The mapped value type.
 * @tparam Container The backing associative container type. Must provide
 *                   the standard map interface (@c operator[], @c find,
 *                   @c contains, @c erase, @c size, @c empty,
 *                   @c begin / @c end).
 */
template <typename KeyId, typename Value,
    typename Container = std::flat_map<KeyId, Value>>
class PropertyMap {
public:
    using KeyType = KeyId;
    using ValueType = Value;
    using ContainerType = Container;

    PropertyMap() = default;

    /**
     * @brief Initialize from a range of keys with a default value.
     *
     * @tparam R A range whose value type is convertible to @p KeyId.
     * @param keys         A range of key IDs (e.g. @c graph.getNodes()).
     * @param defaultValue The value to assign to each key.
     */
    template <std::ranges::input_range R>
        requires std::convertible_to<std::ranges::range_value_t<R>, KeyId>
    PropertyMap(R&& keys, const Value& defaultValue)
    {
        for (auto&& key : keys)
            m_data.insert_or_assign(KeyId(key), defaultValue);
    }

    /**
     * @brief Access element by key
     *
     * @param id The key to access.
     * @return Reference to the mapped value.
     */
    Value& operator[](const KeyId& id) { return m_data[id]; }

    /**
     * @brief Safe const access by key.
     *
     * @param id The key to look up.
     * @return Reference to the value on success, or
     *         @c PropertyMapError::KeyNotFound.
     */
    std::expected<std::reference_wrapper<const Value>, PropertyMapError> get(
        const KeyId& id) const
    {
        auto it = m_data.find(id);
        if (it != m_data.end())
            return std::cref(it->second);
        return std::unexpected(PropertyMapError::KeyNotFound);
    }

    /**
     * @brief Safe mutable access by key.
     *
     * @param id The key to look up.
     * @return Reference to the value on success, or
     *         @c PropertyMapError::KeyNotFound.
     */
    std::expected<std::reference_wrapper<Value>, PropertyMapError> get(
        const KeyId& id)
    {
        auto it = m_data.find(id);
        if (it != m_data.end())
            return std::ref(it->second);
        return std::unexpected(PropertyMapError::KeyNotFound);
    }

    /**
     * @brief Set the value for a key, inserting or overwriting.
     *
     * @param id    The key.
     * @param value The value to store.
     */
    void set(const KeyId& id, Value value)
    {
        m_data.insert_or_assign(id, std::move(value));
    }

    /**
     * @brief Remove the entry for a key.
     *
     * @param id The key to remove.
     * @return Empty expected on success, or
     *         @c PropertyMapError::KeyNotFound.
     */
    std::expected<void, PropertyMapError> erase(const KeyId& id)
    {
        auto count = m_data.erase(id);
        if (count == 0)
            return std::unexpected(PropertyMapError::KeyNotFound);
        return {};
    }

    /** @brief Remove all entries. */
    void clear() { m_data.clear(); }

    // --------------------------------------------------------
    // Query
    // --------------------------------------------------------

    /**
     * @brief Check whether a key has an associated value.
     *
     * @param id The key to check for.
     * @return @c true if @p id has an entry in the map.
     */
    bool contains(const KeyId& id) const { return m_data.contains(id); }

    /**
     * @brief Number of stored key-value pairs.
     *
     * @return The number of entries.
     */
    std::size_t size() const { return m_data.size(); }

    /**
     * @brief Whether the map is empty.
     *
     * @return @c true if the map contains no entries.
     */
    bool empty() const { return m_data.empty(); }

    // --------------------------------------------------------
    // Iteration
    // --------------------------------------------------------

    auto begin() { return m_data.begin(); }
    auto end() { return m_data.end(); }
    auto begin() const { return m_data.begin(); }
    auto end() const { return m_data.end(); }
    auto cbegin() const { return m_data.cbegin(); }
    auto cend() const { return m_data.cend(); }

private:
    Container m_data;
};

// ============================================================
// Convenience aliases
// ============================================================

/**
 * @brief PropertyMap keyed by @c NodeId.
 *
 * @tparam Value     The mapped value type.
 * @tparam Container The backing container (defaults to
 *                   @c std::flat_map<NodeId, Value>).
 */
template <typename Value, typename Container = std::flat_map<NodeId, Value>>
using NodePropertyMap = PropertyMap<NodeId, Value, Container>;

/**
 * @brief PropertyMap keyed by @c EdgeId.
 *
 * @tparam Value     The mapped value type.
 * @tparam Container The backing container (defaults to
 *                   @c std::flat_map<EdgeId, Value>).
 */
template <typename Value, typename Container = std::flat_map<EdgeId, Value>>
using EdgePropertyMap = PropertyMap<EdgeId, Value, Container>;

} // namespace MathUtils

#endif // PROPERTY_MAP_HPP
