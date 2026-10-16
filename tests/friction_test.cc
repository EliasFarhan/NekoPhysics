// Coulomb friction (M3 stage 2): tangent impulses clamped by the accumulated
// normal impulse, combined coefficient sqrt(friction1 * friction2).
#include <array>
#include <cmath>
#include <cstdint>
#include <type_traits>

#include <gtest/gtest.h>

#include "neko/math/angle.h"
#include "neko/math/const.h"
#include "neko/math/func.h"
#include "neko/physics/physics.h"

namespace {

using neko::BodyIndex;
using neko::ColliderIndex;
using neko::Radian;
using neko::Scalar;
using neko::Vec2f;

constexpr Scalar Ratio(int num, int den) {
  return static_cast<Scalar>(num) / static_cast<Scalar>(den);
}

template <typename T>
float ToFloat(T value) {
  if constexpr (std::is_same_v<T, float>) {
    return value;
  } else {
    return static_cast<float>(value);
  }
}

constexpr float kGravity = 9.81f;
constexpr float kDt = 1.0f / 60.0f;

// A static ground (top at y = 0, 200 m wide) with the given friction
struct Ground {
  explicit Ground(Scalar friction) {
    const auto body = world.AddBody();
    world.body(body).type = neko::BodyType::kStatic;
    world.body(body).position = {Scalar{0}, Scalar{-1}};
    const auto collider = world.AddAabbCollider(body);
    world.collider(collider).is_trigger = false;
    world.collider(collider).restitution = Scalar{0};
    world.collider(collider).friction = friction;
    world.aabb(world.collider(collider).shape_index).half_size = {Scalar{100},
                                                                  Scalar{1}};
  }
  // A 1 m box of mass 1 resting on the ground, sliding right at speed
  BodyIndex AddSlidingBox(Scalar friction, Scalar speed) {
    const auto body = world.AddBody();
    world.body(body).position = {Scalar{0}, Ratio(1, 2)};
    world.body(body).velocity = {speed, Scalar{0}};
    const auto collider = world.AddAabbCollider(body);
    world.collider(collider).is_trigger = false;
    world.collider(collider).restitution = Scalar{0};
    world.collider(collider).friction = friction;
    world.aabb(world.collider(collider).shape_index).half_size = {Ratio(1, 2),
                                                                  Ratio(1, 2)};
    return body;
  }
  neko::PhysicsWorld world;
};

struct Slide {
  int steps = 0;
  float distance = 0.0f;
};

// Steps and distance until the box stops (10 s at most)
Slide SlideUntilStop(Ground& ground, BodyIndex box) {
  Slide slide;
  const float start = ToFloat(ground.world.body(box).position.x());
  while (ground.world.body(box).velocity.x() > Scalar{0} && slide.steps < 600) {
    ground.world.Step();
    ++slide.steps;
  }
  slide.distance = ToFloat(ground.world.body(box).position.x()) - start;
  return slide;
}

}  // namespace

TEST(Friction, SlidingBoxStopsAfterVSquaredOverTwoMuG) {
  // Kinetic friction decelerates at mu g: it stops after v / (mu g) seconds
  // and v^2 / (2 mu g) meters. 3 m/s with mu = 0.5: 0.612 s, 0.917 m.
  Ground ground{Ratio(1, 2)};
  const auto box = ground.AddSlidingBox(Ratio(1, 2), Scalar{3});
  const auto slide = SlideUntilStop(ground, box);
  const float mu = 0.5f;
  const float stop_time = 3.0f / (mu * kGravity);
  const float stop_distance = 3.0f * 3.0f / (2.0f * mu * kGravity);
  EXPECT_NEAR(static_cast<float>(slide.steps) * kDt, stop_time, kDt);
  EXPECT_NEAR(slide.distance, stop_distance, 3.0f * kDt);
  // Stopped, not pushed back
  EXPECT_EQ(ground.world.body(box).velocity.x(), Scalar{0});
}

TEST(Friction, FrictionlessBoxKeepsSliding) {
  Ground ground{Scalar{0}};
  const auto box = ground.AddSlidingBox(Ratio(1, 2), Scalar{3});
  for (int step = 0; step < 60; ++step) {
    ground.world.Step();
  }
  EXPECT_EQ(ground.world.body(box).velocity.x(), Scalar{3});
}

TEST(Friction, CoefficientsCombineAsTheGeometricMean) {
  // sqrt(0.25 * 1) = 0.5: the same slide as 0.5 against 0.5
  Ground mixed{Scalar{1}};
  const auto mixed_box = mixed.AddSlidingBox(Ratio(1, 4), Scalar{3});
  Ground even{Ratio(1, 2)};
  const auto even_box = even.AddSlidingBox(Ratio(1, 2), Scalar{3});
  const auto mixed_slide = SlideUntilStop(mixed, mixed_box);
  const auto even_slide = SlideUntilStop(even, even_box);
  EXPECT_EQ(mixed_slide.steps, even_slide.steps);
  EXPECT_NEAR(mixed_slide.distance, even_slide.distance, 0.001f);
}

namespace {
// A 1 m box resting on a static ramp turned by angle (counter-clockwise: the
// ramp goes up to the right, and the box slides down to the left)
struct Ramp {
  Ramp(int degrees, Scalar friction) {
    const Scalar angle =
        Radian{neko::Degree{static_cast<Scalar>(degrees)}}.value();
    const auto ramp = world.AddBody();
    world.body(ramp).type = neko::BodyType::kStatic;
    world.body(ramp).rotation.set_angle(Radian{angle});
    const auto ramp_collider = world.AddOrientedBoxCollider(ramp);
    world.collider(ramp_collider).is_trigger = false;
    world.collider(ramp_collider).restitution = Scalar{0};
    world.collider(ramp_collider).friction = friction;
    world.oriented_box(world.collider(ramp_collider).shape_index).half_size = {
        Scalar{20}, Ratio(1, 2)};
    up = world.body(ramp).rotation.y_axis();
    down_slope = -world.body(ramp).rotation.x_axis();
    box = world.AddBody();
    world.body(box).rotation.set_angle(Radian{angle});
    world.body(box).position = up * Scalar{1};  // 0.5 + 0.5: touching
    const auto box_collider = world.AddOrientedBoxCollider(box);
    world.collider(box_collider).is_trigger = false;
    world.collider(box_collider).restitution = Scalar{0};
    world.collider(box_collider).friction = friction;
    world.oriented_box(world.collider(box_collider).shape_index).half_size = {
        Ratio(1, 2), Ratio(1, 2)};
  }
  float SpeedDownTheSlope() const {
    return ToFloat(Vec2f::Dot(world.body(box).velocity, down_slope));
  }
  neko::PhysicsWorld world;
  BodyIndex box{};
  Vec2f up{};
  Vec2f down_slope{};
};
}  // namespace

TEST(Friction, StaticFrictionHoldsABoxOnAGentleSlope) {
  // tan 20 = 0.36 < 0.5: friction holds the box
  Ramp ramp{20, Ratio(1, 2)};
  const Vec2f start = ramp.world.body(ramp.box).position;
  for (int step = 0; step < 120; ++step) {
    ramp.world.Step();
  }
  const Vec2f moved = ramp.world.body(ramp.box).position - start;
  EXPECT_LT(std::abs(ToFloat(Vec2f::Dot(moved, ramp.down_slope))), 0.01f);
  EXPECT_LT(std::abs(ramp.SpeedDownTheSlope()), 0.01f);
}

TEST(Friction, BoxSlidesDownASteepSlopeAtGTimesSinMinusMuCos) {
  // tan 35 = 0.70 > 0.5: it slides, accelerating at
  // g (sin 35 - 0.5 cos 35) = 1.61 m/s^2
  Ramp ramp{35, Ratio(1, 2)};
  for (int step = 0; step < 60; ++step) {
    ramp.world.Step();
  }
  const float angle = 35.0f * 3.14159265f / 180.0f;
  const float acceleration =
      kGravity * (std::sin(angle) - 0.5f * std::cos(angle));
  const float time = 60.0f * ToFloat(ramp.world.dt());
  EXPECT_NEAR(ramp.SpeedDownTheSlope(), acceleration * time,
              acceleration * time * 0.05f);
  EXPECT_NEAR(ToFloat(ramp.world.body(ramp.box).angular_velocity), 0.0f, 0.05f)
      << "slides flat, doesn't tip";
}

TEST(Friction, SlidingDiskStartsRollingAtTwoThirdsOfItsSpeed) {
  // A disk thrown without spin slides, friction slows it and spins it up
  // until its contact point stops slipping: v = w r. Angular momentum about
  // the contact point is kept, so for a disk (I = m r^2 / 2) the rolling
  // speed is 2/3 of the throw.
  Ground ground{Ratio(1, 2)};
  const auto disk = ground.world.AddBody();
  ground.world.body(disk).position = {Scalar{0}, Ratio(1, 2)};
  ground.world.body(disk).velocity = {Scalar{3}, Scalar{0}};
  const auto collider = ground.world.AddCircleCollider(disk);
  ground.world.collider(collider).is_trigger = false;
  ground.world.collider(collider).restitution = Scalar{0};
  ground.world.collider(collider).friction = Ratio(1, 2);
  ground.world.circle(ground.world.collider(collider).shape_index).radius =
      Ratio(1, 2);
  for (int step = 0; step < 120; ++step) {
    ground.world.Step();
  }
  const auto& body = ground.world.body(disk);
  EXPECT_NEAR(ToFloat(body.velocity.x()), 2.0f, 0.04f);
  // Rolling right turns clockwise: w = -v / r
  EXPECT_NEAR(ToFloat(body.angular_velocity) * 0.5f,
              -ToFloat(body.velocity.x()), 0.02f);
}

TEST(Friction, TangentImpulseStaysWithinTheCoulombLimit) {
  Ground ground{Ratio(1, 2)};
  const auto box = ground.AddSlidingBox(Ratio(1, 2), Scalar{3});
  ground.world.Step();
  // The contact of the last step isn't kept by the world; rebuild it
  neko::Contact contact{};
  auto& ground_collider = ground.world.collider(neko::ColliderIndex{0});
  auto& box_collider = ground.world.collider(neko::ColliderIndex{1});
  ASSERT_TRUE(ground.world.DetectContact(
      ground.world.body(ground_collider.body_index), ground_collider,
      ground.world.body(box), box_collider, &contact));
  std::array<neko::internal::ContactPointScratch, 2> scratch{};
  neko::SolveContacts({&contact, 1}, scratch, {.dt = ground.world.dt()});
  for (std::uint8_t i = 0; i < contact.point_count; ++i) {
    EXPECT_LE(
        neko::Abs(contact.points[i].tangent_impulse),
        contact.friction * contact.points[i].normal_impulse + Ratio(1, 65536));
  }
  EXPECT_EQ(contact.friction, Ratio(1, 2));
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
