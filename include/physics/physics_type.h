#pragma once

#include "container/index_based_container.h"
#include "math/fixed.h"
#include "math/vec2.h"

#include <functional>
#include <variant>

namespace neko
{


enum class ShapeType : short
{
    AABB,
    CIRCLE,
    PLANE,
    NONE
};

enum class BodyType : std::uint8_t
{
    DYNAMIC,
    KINEMATIC,
    STATIC,
    NONE
};

struct Body
{
    Vec2f position{};
    Vec2f velocity{};
    Vec2f force{};
    Scalar inverseMass{1};
    BodyType type = BodyType::DYNAMIC;
    [[nodiscard]] bool IsInvalid() const noexcept { return inverseMass < 0;}
    static Body GenerateInvalidValue() {return {.inverseMass = -1};}

};
using BodyIndex = Index<Body>;

constexpr auto INVALID_BODY_INDEX = BodyIndex{ -1 };

[[nodiscard]] std::uint32_t GenerateChecksum(const Body& body);

struct Collider;
using ColliderIndex = Index<Collider>;
constexpr auto INVALID_COLLIDER_INDEX = ColliderIndex{ -1 };

[[nodiscard]] std::uint32_t GenerateChecksum(const Collider& collider);

struct ColliderPair
{
    ColliderIndex c1;
    ColliderIndex c2;
    constexpr bool operator ==(const ColliderPair& rhs) const noexcept
    {
        return (c1 == rhs.c1 && c2 == rhs.c2) || (c1 == rhs.c2 && c2 == rhs.c1);
    }

    [[nodiscard]] constexpr bool contains(ColliderIndex c) const
    {
        return c == c1 || c == c2;
    }
};

struct ColliderEqual
{
    bool operator()(const ColliderPair& lhs, const ColliderPair& rhs) const
    {
        return lhs == rhs;
    }
};

struct ColliderHash
{
    auto operator()(const ColliderPair& p) const noexcept
    {
        const auto h1 = std::hash<int>{}(p.c1.index());
        const auto h2 = std::hash<int>{}(p.c2.index());
        return h1 + h2;
    }
};
struct CircleCollider
{
    Scalar radius{ 0 };
    [[nodiscard]] bool IsInvalid() const noexcept { return radius < 0; }
    static CircleCollider GenerateInvalidValue() {return {.radius = -1};}
};
using CircleIndex = Index<CircleCollider>;

[[nodiscard]] std::uint32_t GenerateChecksum(const CircleCollider& collider);

struct AabbCollider
{
    Vec2f halfSize{ Scalar {0}, Scalar {0} };
    [[nodiscard]] bool IsInvalid() const noexcept { return halfSize.x < 0; }
    static AabbCollider GenerateInvalidValue() {return {.halfSize = {Scalar {-1}, Scalar {-1}}};}
};
using AabbIndex = Index<AabbCollider>;

[[nodiscard]] std::uint32_t GenerateChecksum(const AabbCollider& collider);

struct PlaneCollider
{
    Vec2f normal{ Vec2f::zero() };
    [[nodiscard]] bool IsInvalid() const noexcept { return normal == Vec2f::zero(); }
    static PlaneCollider GenerateInvalidValue() {return {.normal = Vec2f::zero()};}
};
using PlaneIndex = Index<PlaneCollider>;

class Shape : public std::variant<AabbCollider, CircleCollider, PlaneCollider, std::monostate>
{
    public:
    using std::variant<AabbCollider, CircleCollider, PlaneCollider, std::monostate>::variant;
    [[nodiscard]] bool IsInvalid() const noexcept { return std::holds_alternative<std::monostate>(*this); }
    static Shape GenerateInvalidValue() {Shape newShape = std::monostate{}; return newShape;}
};
using ShapeIndex = Index<Shape>;


constexpr auto INVALID_SHAPE_INDEX = ShapeIndex{-1, 0};
struct Collider
{
    const void* userData = nullptr;
    Vec2f offset{};
    BodyIndex bodyIndex = INVALID_BODY_INDEX;
    ColliderIndex colliderIndex = INVALID_COLLIDER_INDEX;
    ShapeIndex shapeIndex = INVALID_SHAPE_INDEX;
    Scalar restitution{ 1 };
    ShapeType type = ShapeType::NONE;
    bool isTrigger = true;

    [[nodiscard]] bool IsInvalid() const noexcept { return restitution < 0; }
    static Collider GenerateInvalidValue() {return {.restitution = -1};}
};
[[nodiscard]] std::uint32_t GenerateChecksum(const PlaneCollider& collider);







}
