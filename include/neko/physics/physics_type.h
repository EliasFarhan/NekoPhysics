#ifndef NEKO_PHYSICS_PHYSICS_TYPE_H_
#define NEKO_PHYSICS_PHYSICS_TYPE_H_

#include <compare>
#include <cstdint>
#include <limits>
#include <type_traits>

namespace neko
{

/**
 * @brief Integer type of body, shape and collider indices. 16 bits keep snapshots small on the DS.
 */
using Index = std::uint16_t;

class BodyIndex
{
public:
    constexpr BodyIndex() noexcept = default;
    explicit constexpr BodyIndex(Index index_value) noexcept : index_(index_value) {}

    [[nodiscard]] constexpr Index index() const noexcept { return index_; }

    constexpr bool operator==(const BodyIndex& rhs) const noexcept = default;

private:
    Index index_ = std::numeric_limits<Index>::max();
};

static_assert(sizeof(BodyIndex) == sizeof(Index));
static_assert(std::is_trivially_copyable_v<BodyIndex>);

constexpr auto kInvalidBodyIndex = BodyIndex{};

class ShapeIndex
{
public:
    constexpr ShapeIndex() noexcept = default;
    explicit constexpr ShapeIndex(Index index_value) noexcept : index_(index_value) {}

    [[nodiscard]] constexpr Index index() const noexcept { return index_; }

    constexpr bool operator==(const ShapeIndex& rhs) const noexcept = default;

private:
    Index index_ = std::numeric_limits<Index>::max();
};

static_assert(sizeof(ShapeIndex) == sizeof(Index));
static_assert(std::is_trivially_copyable_v<ShapeIndex>);

constexpr auto kInvalidShapeIndex = ShapeIndex{};

enum class ShapeType : std::uint8_t
{
    kAabb,
    kCircle,
    kNone
};

class ColliderIndex
{
public:
    constexpr ColliderIndex() noexcept = default;
    explicit constexpr ColliderIndex(Index index_value) noexcept : index_(index_value) {}

    [[nodiscard]] constexpr Index index() const noexcept { return index_; }

    constexpr auto operator<=>(const ColliderIndex& rhs) const noexcept = default;

private:
    Index index_ = std::numeric_limits<Index>::max();
};

static_assert(sizeof(ColliderIndex) == sizeof(Index));
static_assert(std::is_trivially_copyable_v<ColliderIndex>);

constexpr auto kInvalidColliderIndex = ColliderIndex{};

/**
 * @brief Unordered pair of colliders, stored with c1 < c2 so that it can be compared and sorted
 */
class ColliderPair
{
public:
    constexpr ColliderPair() noexcept = default;
    constexpr ColliderPair(ColliderIndex a, ColliderIndex b) noexcept : c1_(a < b ? a : b), c2_(a < b ? b : a)
    {
    }

    constexpr auto operator<=>(const ColliderPair& rhs) const noexcept = default;

    [[nodiscard]] constexpr ColliderIndex c1() const noexcept { return c1_; }
    [[nodiscard]] constexpr ColliderIndex c2() const noexcept { return c2_; }

    [[nodiscard]] constexpr bool Contains(ColliderIndex c) const noexcept
    {
        return c == c1_ || c == c2_;
    }

private:
    ColliderIndex c1_{};
    ColliderIndex c2_{};
};

static_assert(sizeof(ColliderPair) == 2 * sizeof(Index));
static_assert(std::is_trivially_copyable_v<ColliderPair>);

}

#endif  // NEKO_PHYSICS_PHYSICS_TYPE_H_
