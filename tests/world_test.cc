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
    void OnTriggerEnter(const neko::ColliderPair&) override { ++trigger_enter; }
    void OnTriggerExit(const neko::ColliderPair&) override { ++trigger_exit; }
    void OnCollisionEnter(const neko::ColliderPair&) override { ++collision_enter; }
    void OnCollisionExit(const neko::ColliderPair&) override { ++collision_exit; }
    int trigger_enter = 0;
    int trigger_exit = 0;
    int collision_enter = 0;
    int collision_exit = 0;
};
}  // namespace

TEST(ColliderPair, IsNormalized)
{
    constexpr neko::ColliderPair p1{ neko::ColliderIndex{3}, neko::ColliderIndex{1} };
    constexpr neko::ColliderPair p2{ neko::ColliderIndex{1}, neko::ColliderIndex{3} };
    static_assert(p1 == p2);
    static_assert(p1.c1() == neko::ColliderIndex{ 1 } && p1.c2() == neko::ColliderIndex{ 3 });
    static_assert(neko::ColliderPair{ neko::ColliderIndex{0}, neko::ColliderIndex{5} } < p1);
}

// RemoveBody looped forever: removing a collider didn't clear its body_index, so find_if kept finding it
TEST(PhysicsWorld, RemoveBodyRemovesItsColliders)
{
    neko::PhysicsWorld world{};
    const auto body = world.AddBody();
    const auto circle = world.AddCircleCollider(body);
    const auto aabb = world.AddAabbCollider(body);
    world.RemoveBody(body);
    EXPECT_EQ(world.body(body).type, neko::BodyType::kNone);
    EXPECT_EQ(world.collider(circle).type, neko::ShapeType::kNone);
    EXPECT_EQ(world.collider(aabb).type, neko::ShapeType::kNone);
}

// Removed AABB shapes were never reused (the free test was half_size.x < -1, removal set -1)
TEST(PhysicsWorld, SlotsAreReused)
{
    neko::PhysicsWorld world{};
    const auto body = world.AddBody();
    world.body(body).position = V(3, 4);
    const auto aabb = world.AddAabbCollider(body);
    const auto aabb_shape = world.collider(aabb).shape_index;
    const auto circle = world.AddCircleCollider(body);
    const auto circle_shape = world.collider(circle).shape_index;
    world.RemoveBody(body);

    const auto new_body = world.AddBody();
    EXPECT_EQ(new_body, body);
    // A reused body starts from a clean state
    EXPECT_EQ(world.body(new_body).position, Vec2f::Zero());
    const auto new_aabb = world.AddAabbCollider(new_body);
    EXPECT_EQ(world.collider(new_aabb).shape_index, aabb_shape);
    const auto new_circle = world.AddCircleCollider(new_body);
    EXPECT_EQ(world.collider(new_circle).shape_index, circle_shape);
}

TEST(PhysicsWorld, CapacityIsFixedAtConstruction)
{
    neko::PhysicsWorld world{ {.max_bodies = 2, .max_colliders = 1} };
    const auto b1 = world.AddBody();
    const auto b2 = world.AddBody();
    EXPECT_NE(b1, neko::kInvalidBodyIndex);
    EXPECT_NE(b2, neko::kInvalidBodyIndex);
    EXPECT_EQ(world.AddBody(), neko::kInvalidBodyIndex);

    EXPECT_NE(world.AddCircleCollider(b1), neko::kInvalidColliderIndex);
    EXPECT_EQ(world.AddAabbCollider(b2), neko::kInvalidColliderIndex);

    world.RemoveBody(b1);
    EXPECT_EQ(world.AddBody(), b1);
    EXPECT_NE(world.AddAabbCollider(b1), neko::kInvalidColliderIndex);
}

TEST(PhysicsWorld, CollisionEnterAndExitOnce)
{
    neko::PhysicsWorld world{ {.gravity = Vec2f::Zero()} };
    neko::QuadTree quad_tree;
    CountingListener listener;
    world.SetBSH(&quad_tree);
    world.SetContactListener(&listener);

    const auto a = world.AddBody();
    const auto b = world.AddBody();
    world.body(a).position = V(-1.1f, 0);
    world.body(b).position = V(1.1f, 0);
    world.body(a).velocity = V(3, 0);
    for (const auto body : { a, b })
    {
        const auto collider = world.AddCircleCollider(body);
        world.collider(collider).is_trigger = false;
        world.circle(world.collider(collider).shape_index).radius = Scalar{ 1 };
    }
    for (int i = 0; i < 120; ++i)
    {
        world.Step();
    }
    // a hits b, they exchange velocities and b leaves
    EXPECT_EQ(listener.collision_enter, 1);
    EXPECT_EQ(listener.collision_exit, 1);
    EXPECT_TRUE(world.touching_pairs().empty());
}

TEST(PhysicsWorld, TriggerBecomingSolidEndsTriggerAndStartsCollision)
{
    neko::PhysicsWorld world{ {.gravity = Vec2f::Zero()} };
    neko::QuadTree quad_tree;
    CountingListener listener;
    world.SetBSH(&quad_tree);
    world.SetContactListener(&listener);
    const auto a = world.AddBody();
    const auto b = world.AddBody();
    world.body(b).position = V(0.5f, 0);
    const auto ca = world.AddCircleCollider(a);
    const auto cb = world.AddCircleCollider(b);
    world.circle(world.collider(ca).shape_index).radius = Scalar{ 1 };
    world.circle(world.collider(cb).shape_index).radius = Scalar{ 1 };
    world.collider(cb).is_trigger = false;

    world.Step();
    EXPECT_EQ(listener.trigger_enter, 1);
    world.collider(ca).is_trigger = false;
    world.Step();
    EXPECT_EQ(listener.trigger_exit, 1);
    EXPECT_EQ(listener.collision_enter, 1);
}

// Removing a body while it touches another sends the exit event
TEST(PhysicsWorld, RemovedBodyExits)
{
    neko::PhysicsWorld world{ {.gravity = Vec2f::Zero()} };
    neko::QuadTree quad_tree;
    CountingListener listener;
    world.SetBSH(&quad_tree);
    world.SetContactListener(&listener);
    const auto a = world.AddBody();
    const auto b = world.AddBody();
    world.body(b).position = V(0.5f, 0);
    for (const auto body : { a, b })
    {
        const auto collider = world.AddCircleCollider(body);
        world.circle(world.collider(collider).shape_index).radius = Scalar{ 1 };
    }
    world.Step();
    EXPECT_EQ(listener.trigger_enter, 1);
    world.RemoveBody(b);
    world.Step();
    EXPECT_EQ(listener.trigger_exit, 1);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
