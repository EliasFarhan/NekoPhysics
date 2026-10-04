#include "neko/physics/physics.h"

#include <cmath>

#include <gtest/gtest.h>

#include "neko/bsh/quadtree.h"

namespace {
using neko::Scalar;
using neko::Vec2f;

constexpr float kTolerance = 0.01f;

struct BodyCollider {
  neko::BodyIndex body;
  neko::ColliderIndex collider;
};

BodyCollider AddCircle(neko::PhysicsWorld& world, Vec2f position, Scalar radius,
                       neko::BodyType type = neko::BodyType::kDynamic) {
  const auto body = world.AddBody();
  world.body(body).position = position;
  world.body(body).type = type;
  const auto collider = world.AddCircleCollider(body);
  world.collider(collider).is_trigger = false;
  world.circle(world.collider(collider).shape_index).radius = radius;
  return {body, collider};
}

BodyCollider AddBox(neko::PhysicsWorld& world, Vec2f position, Vec2f half_size,
                    neko::BodyType type = neko::BodyType::kDynamic) {
  const auto body = world.AddBody();
  world.body(body).position = position;
  world.body(body).type = type;
  const auto collider = world.AddAabbCollider(body);
  world.collider(collider).is_trigger = false;
  world.aabb(world.collider(collider).shape_index).half_size = half_size;
  return {body, collider};
}

class CountingListener final : public neko::ContactListener {
 public:
  void OnTriggerEnter(const neko::ColliderPair&) override { ++trigger_enter; }
  void OnTriggerExit(const neko::ColliderPair&) override { ++trigger_exit; }
  void OnCollisionEnter(const neko::ColliderPair&) override {
    ++collision_enter;
  }
  void OnCollisionExit(const neko::ColliderPair&) override { ++collision_exit; }
  int trigger_enter = 0;
  int trigger_exit = 0;
  int collision_enter = 0;
  int collision_exit = 0;
};

class PhysicsWorldTest : public ::testing::Test {
 protected:
  neko::PhysicsWorld world_{{.gravity = Vec2f::Zero()}};
  neko::QuadTree quad_tree_;
  CountingListener listener_;
  void SetUp() override {
    world_.SetBSH(&quad_tree_);
    world_.SetContactListener(&listener_);
  }
};

Vec2f V(float x, float y) { return {Scalar{x}, Scalar{y}}; }
float F(Scalar value) { return static_cast<float>(value); }
}  // namespace

// Gravity is an acceleration: it used to be divided by the inverse mass then
// multiplied back (DS review D2.1)
TEST(Integrator, GravityDoesNotDependOnMass) {
  neko::PhysicsWorld world{};
  const auto light = world.AddBody();
  const auto heavy = world.AddBody();
  const auto infinite = world.AddBody();
  world.body(light).inverse_mass = Scalar{1};
  world.body(heavy).inverse_mass = Scalar{0.25f};
  world.body(infinite).inverse_mass = Scalar{0};
  for (int i = 0; i < 60; ++i) {
    world.Step();
  }
  for (const auto body : {light, heavy, infinite}) {
    EXPECT_NEAR(F(world.body(body).velocity.y()), -9.81f, kTolerance);
    EXPECT_NEAR(F(world.body(body).velocity.x()), 0.0f, kTolerance);
  }
}

TEST(Integrator, ForceIsScaledByInverseMass) {
  neko::PhysicsWorld world{{.gravity = Vec2f::Zero()}};
  const auto body = world.AddBody();
  world.body(body).inverse_mass = Scalar{0.5f};
  world.body(body).force = V(60, 0);
  world.Step();
  EXPECT_NEAR(F(world.body(body).velocity.x()), 60.0f * 0.5f / 60.0f,
              kTolerance);
  EXPECT_EQ(world.body(body).force, Vec2f::Zero());
}

// Collision response used to be skipped when nobody listened to the events
// (review S2.6)
TEST(Collisions, ResolvedWithoutContactListener) {
  neko::PhysicsWorld world{{.gravity = Vec2f::Zero()}};
  neko::QuadTree quad_tree;
  world.SetBSH(&quad_tree);
  const auto a = AddCircle(world, V(-0.9f, 0), Scalar{1});
  const auto b = AddCircle(world, V(0.9f, 0), Scalar{1});
  world.body(a.body).velocity = V(1, 0);
  world.body(b.body).velocity = V(-1, 0);
  world.Step();
  // Equal masses, restitution 1: velocities are exchanged
  EXPECT_NEAR(F(world.body(a.body).velocity.x()), -1.0f, kTolerance);
  EXPECT_NEAR(F(world.body(b.body).velocity.x()), 1.0f, kTolerance);
}

TEST_F(PhysicsWorldTest, HeadOnWithoutRestitution) {
  const auto a = AddCircle(world_, V(-0.9f, 0), Scalar{1});
  const auto b = AddCircle(world_, V(0.9f, 0), Scalar{1});
  world_.collider(a.collider).restitution = Scalar{0};
  world_.collider(b.collider).restitution = Scalar{0};
  world_.body(a.body).velocity = V(1, 0);
  world_.body(b.body).velocity = V(-1, 0);
  world_.Step();
  EXPECT_NEAR(F(world_.body(a.body).velocity.x()), 0.0f, kTolerance);
  EXPECT_NEAR(F(world_.body(b.body).velocity.x()), 0.0f, kTolerance);
  EXPECT_EQ(listener_.collision_enter, 1);
  // Pushed apart until they just touch
  EXPECT_NEAR(
      F(world_.body(b.body).position.x() - world_.body(a.body).position.x()),
      2.0f, kTolerance);
}

// A static body left with the default inverse mass of 1 used to absorb half the
// impulse (review S2.7)
TEST_F(PhysicsWorldTest, StaticBodyHasInfiniteMass) {
  const auto wall = AddBox(world_, V(0, 0), V(1, 1), neko::BodyType::kStatic);
  const auto ball = AddCircle(world_, V(1.5f, 0), Scalar{0.6f});
  world_.body(ball.body).velocity = V(-1, 0);
  world_.Step();
  EXPECT_NEAR(F(world_.body(ball.body).velocity.x()), 1.0f, kTolerance);
  EXPECT_EQ(world_.body(wall.body).position, V(0, 0));
  // The whole penetration goes to the ball
  EXPECT_NEAR(F(world_.body(ball.body).position.x()), 1.6f, kTolerance);
}

TEST_F(PhysicsWorldTest, KinematicBodyHasInfiniteMass) {
  const auto paddle =
      AddBox(world_, V(0, 0), V(1, 1), neko::BodyType::kKinematic);
  const auto ball = AddCircle(world_, V(1.5f, 0), Scalar{0.6f});
  world_.body(ball.body).velocity = V(-1, 0);
  world_.Step();
  EXPECT_NEAR(F(world_.body(ball.body).velocity.x()), 1.0f, kTolerance);
  EXPECT_EQ(world_.body(paddle.body).velocity, Vec2f::Zero());
}

// A circle whose center is inside the box was pushed up by its radius instead
// of out of the nearest face (review S2.8)
TEST_F(PhysicsWorldTest, CircleCenterInsideBoxIsPushedOutOfNearestFace) {
  AddBox(world_, V(0, 0), V(1, 1), neko::BodyType::kStatic);
  const auto ball = AddCircle(world_, V(0.8f, 0.1f), Scalar{0.5f});
  world_.Step();
  EXPECT_NEAR(F(world_.body(ball.body).position.x()), 1.5f, kTolerance);
  EXPECT_NEAR(F(world_.body(ball.body).position.y()), 0.1f, kTolerance);
}

TEST_F(PhysicsWorldTest, CircleCenterInsideBoxIsPushedOutWhenCircleComesFirst) {
  const auto ball = AddCircle(world_, V(-0.1f, -0.7f), Scalar{0.5f});
  AddBox(world_, V(0, 0), V(1, 1), neko::BodyType::kStatic);
  world_.Step();
  EXPECT_NEAR(F(world_.body(ball.body).position.x()), -0.1f, kTolerance);
  EXPECT_NEAR(F(world_.body(ball.body).position.y()), -1.5f, kTolerance);
}

// Circles at the same position used to divide by a zero distance
TEST_F(PhysicsWorldTest, CoincidentCirclesSeparate) {
  const auto a = AddCircle(world_, V(2, 2), Scalar{0.5f});
  const auto b = AddCircle(world_, V(2, 2), Scalar{0.5f});
  world_.Step();
  const auto delta =
      world_.body(b.body).position - world_.body(a.body).position;
  EXPECT_NEAR(F(delta.Length()), 1.0f, kTolerance);
  EXPECT_LT(std::abs(F(world_.body(a.body).position.x())), 10.0f);
  EXPECT_LT(std::abs(F(world_.body(a.body).position.y())), 10.0f);
}

TEST(Collisions, BoxComesToRestOnStaticGround) {
  neko::PhysicsWorld world{};
  neko::QuadTree quad_tree;
  world.SetBSH(&quad_tree);
  const auto ground =
      AddBox(world, V(0, 0), V(5, 0.5f), neko::BodyType::kStatic);
  const auto box = AddBox(world, V(0, 1.5f), V(0.5f, 0.5f));
  // Restitution combines with max, like Box2D
  world.collider(ground.collider).restitution = Scalar{0};
  world.collider(box.collider).restitution = Scalar{0};
  for (int i = 0; i < 120; ++i) {
    world.Step();
  }
  EXPECT_NEAR(F(world.body(box.body).position.y()), 1.0f, 0.02f);
  EXPECT_NEAR(F(world.body(box.body).velocity.y()), 0.0f, kTolerance);
  EXPECT_NEAR(F(world.body(box.body).position.x()), 0.0f, kTolerance);
}

// A pair stored as a trigger dereferenced an empty optional once a collider
// became solid (DS review D2.2)
TEST_F(PhysicsWorldTest, TriggerBecomingSolidIsResolved) {
  const auto a = AddCircle(world_, V(-0.5f, 0), Scalar{1});
  const auto b = AddCircle(world_, V(0.5f, 0), Scalar{1});
  world_.collider(a.collider).is_trigger = true;
  world_.Step();
  EXPECT_EQ(listener_.trigger_enter, 1);
  world_.collider(a.collider).is_trigger = false;
  world_.Step();
  EXPECT_NEAR(
      F(world_.body(b.body).position.x() - world_.body(a.body).position.x()),
      2.0f, kTolerance);
}

// The contact point was on the far side of the first body (review S2.5)
TEST_F(PhysicsWorldTest, ContactPointIsBetweenBodies) {
  const auto c1 = AddCircle(world_, V(0, 0), Scalar{1});
  const auto c2 = AddCircle(world_, V(1.5f, 0), Scalar{1});
  neko::Contact contact{};
  ASSERT_TRUE(world_.DetectContact(
      world_.body(c1.body), world_.collider(c1.collider), world_.body(c2.body),
      world_.collider(c2.collider), &contact));
  EXPECT_NEAR(F(contact.contact_point.x()), 0.75f, kTolerance);

  const auto b1 = AddBox(world_, V(10, 0), V(1, 1));
  const auto b2 = AddBox(world_, V(11.5f, 0), V(1, 1));
  ASSERT_TRUE(world_.DetectContact(
      world_.body(b1.body), world_.collider(b1.collider), world_.body(b2.body),
      world_.collider(b2.collider), &contact));
  EXPECT_NEAR(F(contact.contact_point.x()), 10.75f, kTolerance);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
