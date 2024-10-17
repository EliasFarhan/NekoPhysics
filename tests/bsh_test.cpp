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
    MOCK_METHOD(const ArrayList<neko::ColliderPair>&, GetPossiblePairs,(), (const, override));
private:
};

class MockContactListener : public neko::ContactListener
{
public:
    MOCK_METHOD(void, OnTriggerEnter, (const neko::ColliderPair&), (override));
    MOCK_METHOD(void, OnTriggerExit, (const neko::ColliderPair&), (override));
    MOCK_METHOD(void, OnCollisionEnter, (const neko::ColliderPair&), (override));
    MOCK_METHOD(void, OnCollisionExit, (const neko::ColliderPair&), (override));
};

TEST(PhysicsWorld, TwoBodiesOneFrame)
{
    neko::PhysicsWorld world{neko::Vec2f::zero()};

    const auto b1 = world.AddBody();
    {
        auto& body1 = world.body(b1);
        body1.inverseMass = neko::Scalar{ 1 };
    }
    const auto b2 = world.AddBody();
    {
        auto& body2 = world.body(b2);
        body2.inverseMass = neko::Scalar{ 1 };
    }
    const auto c1 = world.AddAabbCollider(b1);
    {
        const auto& collider1 = world.collider(c1);
        auto& [halfSize] = world.aabb(collider1.shapeIndex);
        halfSize = {neko::Scalar{1}, neko::Scalar{1} };
    }
    const auto c2 = world.AddAabbCollider(b2);
    {
        const auto& collider2 = world.collider(c2);
        auto& [halfSize] = world.aabb(collider2.shapeIndex);
        halfSize = { neko::Scalar{1}, neko::Scalar{1} };
    }

    neko::HeapAllocator allocator_;
    ArrayList<neko::ColliderPair> pairs((neko::StandardAllocator<neko::ColliderPair>(allocator_)));
	pairs.emplace_back(c1, c2);

    MockBSH bsh;
    //Order for Clear and SetWorldAabb is not constrained, but need to be called before Insert
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
    world.Step(neko::Scalar{ 0.1f });
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}