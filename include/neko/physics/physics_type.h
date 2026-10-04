#ifndef NEKO_PHYSICS_PHYSICS_TYPE_H_
#define NEKO_PHYSICS_PHYSICS_TYPE_H_

#include <compare>
#include <cstdint>
#include <limits>

namespace neko
{

/**
 * @brief Integer type of body, shape and collider indices. 16 bits keep snapshots small on the DS.
 */
using Index = std::uint16_t;

struct BodyIndex
{
    Index index = std::numeric_limits<Index>::max();
    constexpr bool operator==(const BodyIndex& rhs) const noexcept = default;
};

constexpr auto INVALID_BODY_INDEX = BodyIndex{};

struct ShapeIndex
{
    Index index = std::numeric_limits<Index>::max();
    constexpr bool operator==(const ShapeIndex& rhs) const noexcept = default;
};

constexpr auto INVALID_SHAPE_INDEX = ShapeIndex{};

enum class ShapeType : std::uint8_t
{
    AABB,
    CIRCLE,
    NONE
};

struct ColliderIndex
{
    Index index = std::numeric_limits<Index>::max();
    constexpr auto operator<=>(const ColliderIndex& rhs) const noexcept = default;
};

constexpr auto INVALID_COLLIDER_INDEX = ColliderIndex{};

/**
 * @brief Unordered pair of colliders, stored with c1 < c2 so that it can be compared and sorted
 */
struct ColliderPair
{
    ColliderIndex c1{};
    ColliderIndex c2{};

    constexpr ColliderPair() noexcept = default;
    constexpr ColliderPair(ColliderIndex a, ColliderIndex b) noexcept : c1(a < b ? a : b), c2(a < b ? b : a)
    {
    }

    constexpr auto operator<=>(const ColliderPair& rhs) const noexcept = default;

    [[nodiscard]] constexpr bool contains(ColliderIndex c) const noexcept
    {
        return c == c1 || c == c2;
    }
};

}

#endif  // NEKO_PHYSICS_PHYSICS_TYPE_H_
