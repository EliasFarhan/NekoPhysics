#include "neko/physics/physics.h"
#include "neko/bsh/quadtree.h"

#include <gtest/gtest.h>

namespace
{
using neko::Scalar;
using neko::Vec2f;

Vec2f V(float x, float y) { return { Scalar{x}, Scalar{y} }; }

class CountingListener final : public neko::ContactListener
{
public:
    void OnTriggerEnter(const neko::ColliderPair&) override { ++triggerEnter; }
    void OnTriggerExit(const neko::ColliderPair&) override { ++triggerExit; }
    void OnCollisionEnter(const neko::ColliderPair&) override { ++collisionEnter; }
    void OnCollisionExit(const neko::ColliderPair&) override { ++collisionExit; }
    int triggerEnter = 0;
    int triggerExit = 0;
    int collisionEnter = 0;
    int collisionExit = 0;
};
} // namespace

TEST(ColliderPair, IsNormalized)
{
    constexpr neko::ColliderPair p1{ neko::ColliderIndex{3}, neko::ColliderIndex{1} };
    constexpr neko::ColliderPair p2{ neko::ColliderIndex{1}, neko::ColliderIndex{3} };
    static_assert(p1 == p2);
    static_assert(p1.c1 == neko::ColliderIndex{ 1 } && p1.c2 == neko::ColliderIndex{ 3 });
    static_assert(neko::ColliderPair{ neko::ColliderIndex{0}, neko::ColliderIndex{5} } < p1);
}

// RemoveBody looped forever: removing a collider didn't clear its bodyIndex, so find_if kept finding it
TEST(PhysicsWorld, RemoveBodyRemovesItsColliders)
{
    neko::PhysicsWorld world{};
    const auto body = world.AddBody();
    const auto circle = world.AddCircleCollider(body);
    const auto aabb = world.AddAabbCollider(body);
    world.RemoveBody(body);
    EXPECT_EQ(world.body(body).type, neko::BodyType::NONE);
    EXPECT_EQ(world.collider(circle).type, neko::ShapeType::NONE);
    EXPECT_EQ(world.collider(aabb).type, neko::ShapeType::NONE);
}

// Removed AABB shapes were never reused (the free test was halfSize.x < -1, removal set -1)
TEST(PhysicsWorld, SlotsAreReused)
{
    neko::PhysicsWorld world{};
    const auto body = world.AddBody();
    world.body(body).position = V(3, 4);
    const auto aabb = world.AddAabbCollider(body);
    const auto aabbShape = world.collider(aabb).shapeIndex;
    const auto circle = world.AddCircleCollider(body);
    const auto circleShape = world.collider(circle).shapeIndex;
    world.RemoveBody(body);

    const auto newBody = world.AddBody();
    EXPECT_EQ(newBody, body);
    // A reused body starts from a clean state
    EXPECT_EQ(world.body(newBody).position, Vec2f::zero());
    const auto newAabb = world.AddAabbCollider(newBody);
    EXPECT_EQ(world.collider(newAabb).shapeIndex, aabbShape);
    const auto newCircle = world.AddCircleCollider(newBody);
    EXPECT_EQ(world.collider(newCircle).shapeIndex, circleShape);
}

TEST(PhysicsWorld, CapacityIsFixedAtConstruction)
{
    neko::PhysicsWorld world{ {.maxBodies = 2, .maxColliders = 1} };
    const auto b1 = world.AddBody();
    const auto b2 = world.AddBody();
    EXPECT_NE(b1, neko::INVALID_BODY_INDEX);
    EXPECT_NE(b2, neko::INVALID_BODY_INDEX);
    EXPECT_EQ(world.AddBody(), neko::INVALID_BODY_INDEX);

    EXPECT_NE(world.AddCircleCollider(b1), neko::INVALID_COLLIDER_INDEX);
    EXPECT_EQ(world.AddAabbCollider(b2), neko::INVALID_COLLIDER_INDEX);

    world.RemoveBody(b1);
    EXPECT_EQ(world.AddBody(), b1);
    EXPECT_NE(world.AddAabbCollider(b1), neko::INVALID_COLLIDER_INDEX);
}

TEST(PhysicsWorld, CollisionEnterAndExitOnce)
{
    neko::PhysicsWorld world{ {.gravity = Vec2f::zero()} };
    neko::QuadTree quadTree;
    CountingListener listener;
    world.SetBSH(&quadTree);
    world.SetContactListener(&listener);

    const auto a = world.AddBody();
    const auto b = world.AddBody();
    world.body(a).position = V(-1.1f, 0);
    world.body(b).position = V(1.1f, 0);
    world.body(a).velocity = V(3, 0);
    for (const auto body : { a, b })
    {
        const auto collider = world.AddCircleCollider(body);
        world.collider(collider).isTrigger = false;
        world.circle(world.collider(collider).shapeIndex).radius = Scalar{ 1 };
    }
    for (int i = 0; i < 120; ++i)
    {
        world.Step();
    }
    // a hits b, they exchange velocities and b leaves
    EXPECT_EQ(listener.collisionEnter, 1);
    EXPECT_EQ(listener.collisionExit, 1);
    EXPECT_TRUE(world.touchingPairs().empty());
}

TEST(PhysicsWorld, TriggerBecomingSolidEndsTriggerAndStartsCollision)
{
    neko::PhysicsWorld world{ {.gravity = Vec2f::zero()} };
    neko::QuadTree quadTree;
    CountingListener listener;
    world.SetBSH(&quadTree);
    world.SetContactListener(&listener);
    const auto a = world.AddBody();
    const auto b = world.AddBody();
    world.body(b).position = V(0.5f, 0);
    const auto ca = world.AddCircleCollider(a);
    const auto cb = world.AddCircleCollider(b);
    world.circle(world.collider(ca).shapeIndex).radius = Scalar{ 1 };
    world.circle(world.collider(cb).shapeIndex).radius = Scalar{ 1 };
    world.collider(cb).isTrigger = false;

    world.Step();
    EXPECT_EQ(listener.triggerEnter, 1);
    world.collider(ca).isTrigger = false;
    world.Step();
    EXPECT_EQ(listener.triggerExit, 1);
    EXPECT_EQ(listener.collisionEnter, 1);
}

// Removing a body while it touches another sends the exit event
TEST(PhysicsWorld, RemovedBodyExits)
{
    neko::PhysicsWorld world{ {.gravity = Vec2f::zero()} };
    neko::QuadTree quadTree;
    CountingListener listener;
    world.SetBSH(&quadTree);
    world.SetContactListener(&listener);
    const auto a = world.AddBody();
    const auto b = world.AddBody();
    world.body(b).position = V(0.5f, 0);
    for (const auto body : { a, b })
    {
        const auto collider = world.AddCircleCollider(body);
        world.circle(world.collider(collider).shapeIndex).radius = Scalar{ 1 };
    }
    world.Step();
    EXPECT_EQ(listener.triggerEnter, 1);
    world.RemoveBody(b);
    world.Step();
    EXPECT_EQ(listener.triggerExit, 1);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
