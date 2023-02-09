#pragma once

#include "vec2.h"

#include <vector>

namespace neko
{

using Scalar = float;
using Vec2f = Vec2<Scalar>;

enum class BodyType
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
    Scalar mass{};
    BodyType type = BodyType::DYNAMIC;
};

struct BodyIndex
{
    std::size_t index;
};

class PhysicsWorld
{
public:
    PhysicsWorld() = default;
    explicit PhysicsWorld(Vec2f gravity);
    BodyIndex AddBody();
    void RemoveBody(BodyIndex index);
    void Step(Scalar dt);
    void Clear();

    [[nodiscard]] Body& body(BodyIndex index) { return bodies_[index.index]; }
    [[nodiscard]] const Body& body(BodyIndex index) const { return bodies_[index.index]; }
private:
    std::vector<Body> bodies_;
    static constexpr Vec2f defaultGravity{0.0f, -9.81f};
    Vec2f gravity_;

};

} // namespace neko
