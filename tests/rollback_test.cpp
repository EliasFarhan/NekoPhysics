#include "physics/physics.h"
#include "bsh/quadtree.h"

#include <gtest/gtest.h>

#include <cstdlib>
#include <vector>

namespace
{
using neko::Scalar;
using neko::Vec2f;

class CountingAllocator final : public neko::Allocator
{
public:
    void* Allocate(std::size_t count, std::size_t typeSize) override
    {
        ++allocationCount;
        return std::malloc(count * typeSize);
    }
    void Deallocate(void* ptr) override { std::free(ptr); }
    int allocationCount = 0;
};

class CountingListener final : public neko::ContactListener
{
public:
    void OnTriggerEnter(const neko::ColliderPair&) override { ++events; }
    void OnTriggerExit(const neko::ColliderPair&) override { ++events; }
    void OnCollisionEnter(const neko::ColliderPair&) override { ++events; }
    void OnCollisionExit(const neko::ColliderPair&) override { ++events; }
    int events = 0;
};

constexpr neko::PhysicsWorldConfig config{ .maxBodies = 32, .maxColliders = 32, .maxTouchingPairs = 128 };

/**
 * @brief A static ground, a pile of falling boxes and circles (some of them triggers) and a kinematic paddle
 */
void BuildScene(neko::PhysicsWorld& world)
{
    const auto ground = world.AddBody();
    world.body(ground).type = neko::BodyType::STATIC;
    world.body(ground).position = { Scalar{0}, Scalar{-1} };
    const auto groundCollider = world.AddAabbCollider(ground);
    world.collider(groundCollider).isTrigger = false;
    world.collider(groundCollider).restitution = Scalar{ 0.5f };
    world.aabb(world.collider(groundCollider).shapeIndex).halfSize = { Scalar{10}, Scalar{1} };

    for (int i = 0; i < 16; ++i)
    {
        const auto body = world.AddBody();
        world.body(body).position = { Scalar{-4.0f + 0.55f * static_cast<float>(i)}, Scalar{1.0f + 0.3f * static_cast<float>(i % 5)} };
        world.body(body).velocity = { Scalar{static_cast<float>(i % 3) - 1.0f}, Scalar{0} };
        const auto collider = i % 2 == 0 ? world.AddCircleCollider(body) : world.AddAabbCollider(body);
        world.collider(collider).isTrigger = i % 7 == 3;
        world.collider(collider).restitution = Scalar{ 0.2f };
        if (i % 2 == 0)
        {
            world.circle(world.collider(collider).shapeIndex).radius = Scalar{ 0.3f };
        }
        else
        {
            world.aabb(world.collider(collider).shapeIndex).halfSize = { Scalar{0.3f}, Scalar{0.25f} };
        }
    }

    const auto paddle = world.AddBody();
    world.body(paddle).type = neko::BodyType::KINEMATIC;
    world.body(paddle).position = { Scalar{-6}, Scalar{0.5f} };
    world.body(paddle).velocity = { Scalar{2}, Scalar{0} };
    const auto paddleCollider = world.AddAabbCollider(paddle);
    world.collider(paddleCollider).isTrigger = false;
    world.aabb(world.collider(paddleCollider).shapeIndex).halfSize = { Scalar{0.2f}, Scalar{0.5f} };
}
} // namespace

TEST(Checksum, WorldChecksumFollowsState)
{
    neko::PhysicsWorld a{ config };
    neko::PhysicsWorld b{ config };
    BuildScene(a);
    BuildScene(b);
    EXPECT_EQ(a.GenerateChecksum(), b.GenerateChecksum());
    b.body({ 3 }).position.x += Scalar{ 0.001f };
    EXPECT_NE(a.GenerateChecksum(), b.GenerateChecksum());
}

// Rollback: restore a snapshot, resimulate, and get exactly the same states and events
TEST(Rollback, ResimulationIsIdentical)
{
    neko::QuadTree quadTree;
    CountingListener listener;
    neko::PhysicsWorld world{ config };
    world.SetBSH(&quadTree);
    world.SetContactListener(&listener);
    BuildScene(world);
    for (int i = 0; i < 30; ++i)
    {
        world.Step();
    }
    ASSERT_FALSE(world.touchingPairs().empty());

    neko::PhysicsWorld snapshot{ config };
    snapshot.CopyFrom(world);
    EXPECT_EQ(snapshot.GenerateChecksum(), world.GenerateChecksum());

    std::vector<std::uint32_t> checksums;
    listener.events = 0;
    for (int i = 0; i < 90; ++i)
    {
        world.Step();
        checksums.push_back(world.GenerateChecksum());
    }
    const int events = listener.events;
    EXPECT_GT(events, 0);

    world.CopyFrom(snapshot);
    listener.events = 0;
    for (int i = 0; i < 90; ++i)
    {
        world.Step();
        EXPECT_EQ(world.GenerateChecksum(), checksums[static_cast<std::size_t>(i)]) << "step " << i;
    }
    EXPECT_EQ(listener.events, events);
}

// Allocate at init only: Step (default broadphase included) and CopyFrom must never allocate (DS review D3.1)
TEST(Rollback, StepAndCopyFromDoNotAllocate)
{
    CountingAllocator allocator;
    auto worldConfig = config;
    worldConfig.allocator = &allocator;
    CountingListener listener;
    neko::PhysicsWorld world{ worldConfig };
    neko::PhysicsWorld snapshot{ worldConfig };
    world.SetContactListener(&listener);
    BuildScene(world);
    world.Step();
    snapshot.CopyFrom(world);

    const int allocationsAfterSetup = allocator.allocationCount;
    for (int i = 0; i < 120; ++i)
    {
        world.Step();
        if (i % 10 == 0)
        {
            snapshot.CopyFrom(world);
        }
        if (i % 30 == 0)
        {
            world.CopyFrom(snapshot);
        }
    }
    EXPECT_GT(listener.events, 0);
    EXPECT_EQ(allocator.allocationCount, allocationsAfterSetup);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
