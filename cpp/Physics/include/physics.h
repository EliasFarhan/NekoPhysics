#pragma once

#include "vec2.h"

#include <vector>

namespace neko
{

using Scalar = float;
using Vec2f = Vec2<Scalar>;

struct Body
{
    Vec2f position{};
    Vec2f velocity{};
    Vec2f force{};
    Scalar mass{};
};

class PhysicsWorld
{
public:
    void Step(Scalar dt);
private:
    std::vector<Body> bodies_;
    static constexpr Vec2f gravity{0.0f, -9.81f};
};

} // namespace neko
