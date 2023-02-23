#include "bsh/bsh.h"

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "physics/contact_listener.h"
#include "physics/physics.h"

class MockBSH : public neko::BoundingSurfaceHierarchy
{
public:
    MOCK_METHOD(void, Insert, (const neko::ColliderAabb&), (override));
    MOCK_METHOD(void, CalculatePairs, (), (override));
    MOCK_METHOD(void, Clear, (), (override));
    MOCK_METHOD(void, SetWorldAabb, (const neko::Aabbf&), (override) );
    MOCK_METHOD( const std::vector<neko::TriggerPair>&, GetPossiblePairs,(), (const, override));
};

class MockContactListener : public neko::ContactListener
{
public:
    MOCK_METHOD(void, OnTriggerEnter, (const neko::TriggerPair&), (override));
    MOCK_METHOD(void, OnTriggerExit, (const neko::TriggerPair&), (override));
};

TEST(PhysicsWorld, TwoBodiesOneFrame)
{
    neko::PhysicsWorld world{neko::Vec2f::zero()};

    const auto b1 = world.AddBody();
    const auto b2 = world.AddBody();

    const auto c1 = world.AddAabbCollider(b1);
    {
        const auto& collider1 = world.collider(c1);
        auto& [halfSize] = world.aabb(collider1.shapeIndex);
        halfSize = { 1.0f, 1.0f };
    }
    const auto c2 = world.AddAabbCollider(b2);
    {
        const auto& collider2 = world.collider(c2);
        auto& [halfSize] = world.aabb(collider2.shapeIndex);
        halfSize = { 1.0f, 1.0f };
    }

    std::vector<neko::TriggerPair> pairs{ {c1, c2} };

    MockBSH bsh;
    testing::Sequence s1, s2;
    EXPECT_CALL(bsh, Clear)
        .Times(1)
        .InSequence(s1);
    EXPECT_CALL(bsh, SetWorldAabb)
        .Times(1)
        .InSequence(s2);
    EXPECT_CALL(bsh, Insert)
        .Times(2)
        .InSequence(s1, s2);
    EXPECT_CALL(bsh, CalculatePairs)
        .Times(1)
        .InSequence(s1, s2);
    EXPECT_CALL(bsh, GetPossiblePairs)
        .Times(1)
        .InSequence(s1, s2)
        .WillOnce(testing::ReturnRef(pairs));
    world.SetBSH(&bsh);

    MockContactListener contactListener;
    EXPECT_CALL(contactListener, OnTriggerEnter)
        .Times(1)
        .InSequence(s1, s2);
    world.SetContactListener(&contactListener);

    //Actual Action
    world.Step(0.1f);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}