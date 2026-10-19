// Off-center collision impulses (M5 stage 4): contacts with an oriented box
// apply their impulse at each manifold point, with lever arms, so an
// off-center hit makes a body spin.
#include <array>
#include <cmath>
#include <type_traits>

#include <gtest/gtest.h>

#include "neko/math/angle.h"
#include "neko/math/const.h"
#include "neko/math/func.h"
#include "neko/math/scalar_traits.h"
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
  return neko::ScalarTraits<T>::ToFloat(value);
}

Scalar Degrees(int degrees) {
  return Radian{neko::Degree{static_cast<Scalar>(degrees)}}.value();
}

constexpr Vec2f kBoxHalfSize{Scalar{1}, Ratio(1, 2)};

// A static ground with its top at y = 0 and a 2 x 1 box of mass 1, without
// gravity. The contact is detected and resolved by hand, like the
// narrowphase does (ground first).
struct Fixture {
  Fixture() {
    ground_body = world.AddBody();
    world.body(ground_body).type = neko::BodyType::kStatic;
    world.body(ground_body).position = {Scalar{0}, Scalar{-1}};
    ground = world.AddAabbCollider(ground_body);
    world.collider(ground).is_trigger = false;
    // A contact bounces with the larger restitution of its two colliders:
    // 0 here lets the box's restitution decide
    world.collider(ground).restitution = Scalar{0};
    // Frictionless: these are normal impulse experiments
    world.collider(ground).friction = Scalar{0};
    world.aabb(world.collider(ground).shape_index).half_size = {Scalar{10},
                                                                Scalar{1}};
  }
  // A box turned by angle whose lowest bottom corner is depth below the
  // ground, falling at velocity
  BodyIndex AddBox(Scalar angle, Scalar depth, Vec2f velocity,
                   Scalar restitution, bool oriented = true) {
    const auto body = world.AddBody();
    world.body(body).rotation.set_angle(Radian{angle});
    const auto& rotation = world.body(body).rotation;
    const Scalar left =
        rotation.Rotate({-kBoxHalfSize.x(), -kBoxHalfSize.y()}).y();
    const Scalar right =
        rotation.Rotate({kBoxHalfSize.x(), -kBoxHalfSize.y()}).y();
    world.body(body).position = {Scalar{0}, -neko::Min(left, right) - depth};
    world.body(body).velocity = velocity;
    box = oriented ? world.AddOrientedBoxCollider(body)
                   : world.AddAabbCollider(body);
    world.collider(box).is_trigger = false;
    world.collider(box).restitution = restitution;
    if (oriented) {
      world.oriented_box(world.collider(box).shape_index).half_size =
          kBoxHalfSize;
    } else {
      world.aabb(world.collider(box).shape_index).half_size = kBoxHalfSize;
    }
    // Detect and resolve by hand don't run Step, which computes the inertia
    world.body(body).inverse_inertia = world.ComputeInverseInertia(body);
    return body;
  }
  // Detects the contact and solves it alone. No Baumgarte push-out: these
  // are pure impulse experiments, the small overlap must not add speed.
  neko::Contact Collide(int iterations = 8) {
    neko::Contact contact{};
    auto& ground_collider = world.collider(ground);
    auto& box_collider = world.collider(box);
    EXPECT_TRUE(world.DetectContact(world.body(ground_body), ground_collider,
                                    world.body(box_collider.body_index),
                                    box_collider, &contact));
    std::array<neko::internal::ContactPointScratch, 2> scratch{};
    neko::internal::SolveContacts({&contact, 1}, scratch,
                                  {.dt = world.dt(),
                                   .velocity_iterations = iterations,
                                   .baumgarte = Scalar{0}});
    return contact;
  }

  neko::PhysicsWorld world{{.gravity = Vec2f::Zero()}};
  BodyIndex ground_body{};
  ColliderIndex ground{};
  ColliderIndex box{};
};

struct CornerHit {
  float lever = 0.0f;  // Cross(r, up) = r.x
  float velocity_y = 0.0f;
  float angular_velocity = 0.0f;
  float point_velocity_y = 0.0f;  // velocity of the corner after the hit
};

// The analytic single-point impact of a box of mass 1 and inertia I falling
// at speed (0, vy) onto a corner at r from its center: j = -(1 + e) vy / k
// with k = 1/m + (r x n)^2 / I
CornerHit PredictCornerHit(Vec2f r, float vy, float restitution) {
  const float inertia = (2.0f * 2.0f + 1.0f * 1.0f) / 12.0f;
  const float lever = ToFloat(r.x());  // Cross(r, (0, 1))
  const float k = 1.0f + lever * lever / inertia;
  const float impulse = -(1.0f + restitution) * vy / k;
  CornerHit hit;
  hit.lever = lever;
  hit.velocity_y = vy + impulse;
  hit.angular_velocity = lever * impulse / inertia;
  hit.point_velocity_y = hit.velocity_y + hit.angular_velocity * lever;
  return hit;
}

}  // namespace

TEST(OffCenterImpulse, CornerHitMatchesTheAnalyticImpulse) {
  for (const int restitution_percent : {0, 50, 100}) {
    Fixture f;
    const Scalar restitution = Ratio(restitution_percent, 100);
    const auto body = f.AddBox(Degrees(20), Ratio(1, 100),
                               {Scalar{0}, Scalar{-2}}, restitution);
    const auto contact = f.Collide();
    ASSERT_EQ(contact.point_count, 1);
    const Vec2f r = contact.points[0].position - f.world.body(body).position;
    const auto predicted = PredictCornerHit(
        r, -2.0f, static_cast<float>(restitution_percent) / 100.0f);
    const auto& box = f.world.body(body);
    EXPECT_NEAR(ToFloat(box.velocity.y()), predicted.velocity_y, 0.002f)
        << restitution_percent;
    EXPECT_NEAR(ToFloat(box.velocity.x()), 0.0f, 0.0001f);
    EXPECT_NEAR(ToFloat(box.angular_velocity), predicted.angular_velocity,
                0.002f)
        << restitution_percent;
    // The corner leaves at -e times its approach speed
    const float corner_velocity =
        ToFloat(box.velocity.y()) +
        ToFloat(box.angular_velocity) * ToFloat(r.x());
    EXPECT_NEAR(corner_velocity,
                2.0f * static_cast<float>(restitution_percent) / 100.0f,
                0.003f);
  }
}

TEST(OffCenterImpulse, LandingCornerDecidesTheSpin) {
  // Turned counter-clockwise, the box lands on its left corner: the push up
  // left of the center turns it clockwise, back toward flat
  Fixture left;
  const auto left_box = left.AddBox(Degrees(20), Ratio(1, 100),
                                    {Scalar{0}, Scalar{-2}}, Scalar{0});
  left.Collide();
  EXPECT_LT(left.world.body(left_box).angular_velocity, Scalar{0});
  Fixture right;
  const auto right_box = right.AddBox(Degrees(-20), Ratio(1, 100),
                                      {Scalar{0}, Scalar{-2}}, Scalar{0});
  right.Collide();
  EXPECT_GT(right.world.body(right_box).angular_velocity, Scalar{0});
  EXPECT_NEAR(ToFloat(left.world.body(left_box).angular_velocity),
              -ToFloat(right.world.body(right_box).angular_velocity), 0.002f);
}

TEST(OffCenterImpulse, ElasticCornerHitKeepsTheKineticEnergy) {
  Fixture f;
  const auto body =
      f.AddBox(Degrees(30), Ratio(1, 100), {Scalar{0}, Scalar{-2}}, Scalar{1});
  f.Collide();
  const auto& box = f.world.body(body);
  const float inertia = 5.0f / 12.0f;
  const float vx = ToFloat(box.velocity.x());
  const float vy = ToFloat(box.velocity.y());
  const float w = ToFloat(box.angular_velocity);
  // 1/2 m v^2 before (m = 1, v = 2), linear plus rotational after
  const float before = 0.5f * 2.0f * 2.0f;
  const float after = 0.5f * (vx * vx + vy * vy) + 0.5f * inertia * w * w;
  EXPECT_NEAR(after, before, before * 0.005f);
  EXPECT_LT(vy, 2.0f) << "part of the energy went into spin";
}

TEST(OffCenterImpulse, HitBelowTheCenterDoesntSpin) {
  // A diamond lands on the corner right below its center: no lever arm, the
  // same bounce as a centered contact
  Fixture f;
  const auto body = f.world.AddBody();
  f.world.body(body).rotation.set_angle(Radian{Degrees(45)});
  f.world.body(body).position = {Scalar{0}, Ratio(141, 100)};
  f.world.body(body).velocity = {Scalar{0}, Scalar{-2}};
  f.box = f.world.AddOrientedBoxCollider(body);
  f.world.collider(f.box).is_trigger = false;
  f.world.collider(f.box).restitution = Ratio(1, 2);
  f.world.oriented_box(f.world.collider(f.box).shape_index).half_size = {
      Scalar{1}, Scalar{1}};
  f.world.body(body).inverse_inertia = f.world.ComputeInverseInertia(body);
  f.Collide();
  EXPECT_NEAR(ToFloat(f.world.body(body).angular_velocity), 0.0f, 0.002f);
  EXPECT_NEAR(ToFloat(f.world.body(body).velocity.y()), 1.0f, 0.002f);
}

TEST(OffCenterImpulse, OnePassRocksAFlatLandingAndIterationsSettleIt) {
  // A flat box touching the ground (two points) falling at 1 m/s with no
  // bounce. One pass: point 0's impulse turns the box, point 1's corrects
  // that, and the box is left rocking. More passes, with the accumulated
  // impulses clamped, converge to rest.
  Fixture one_pass;
  const auto rocking =
      one_pass.AddBox(Scalar{0}, Scalar{0}, {Scalar{0}, Scalar{-1}}, Scalar{0});
  const auto contact = one_pass.Collide(1);
  ASSERT_EQ(contact.point_count, 2);
  const auto& box = one_pass.world.body(rocking);
  EXPECT_LT(box.velocity.y(), Scalar{0}) << "still falling after one pass";
  EXPECT_GT(box.velocity.y(), Scalar{-1});
  EXPECT_NE(box.angular_velocity, Scalar{0}) << "rocking";

  Fixture many_passes;
  const auto settled = many_passes.AddBox(Scalar{0}, Scalar{0},
                                          {Scalar{0}, Scalar{-1}}, Scalar{0});
  const auto solved = many_passes.Collide(30);
  const auto& rest = many_passes.world.body(settled);
  EXPECT_NEAR(ToFloat(rest.velocity.y()), 0.0f, 0.01f);
  EXPECT_NEAR(ToFloat(rest.angular_velocity), 0.0f, 0.01f);
  // The 1 kg box's 1 N s of momentum, shared by the two points
  EXPECT_NEAR(ToFloat(solved.points[0].normal_impulse), 0.5f, 0.01f);
  EXPECT_NEAR(ToFloat(solved.points[1].normal_impulse), 0.5f, 0.01f);
}

TEST(OffCenterImpulse, AabbContactsKeepTheCenterOfMassImpulse) {
  // An AABB landing off-center: its overlap with the ground is a 2 m edge,
  // but its shape doesn't turn, so it keeps the old response (no spin), even
  // though its body has an inertia
  Fixture f;
  const auto body = f.AddBox(Scalar{0}, Ratio(1, 100), {Scalar{0}, Scalar{-2}},
                             Ratio(1, 2), false);
  f.world.body(body).position.set_x(Scalar{9});  // hangs past the ground end
  ASSERT_GT(f.world.body(body).inverse_inertia, Scalar{0});
  f.Collide();
  EXPECT_EQ(f.world.body(body).angular_velocity, Scalar{0});
  EXPECT_EQ(f.world.body(body).velocity.y(), Scalar{1});
}

TEST(OffCenterImpulse, CircleHittingABoxEdgeSpinsTheBoxNotTheCircle) {
  // A ball falls on the right end of a free-floating plank: the plank turns
  // (its point of contact is off its center), the ball doesn't (the normal
  // goes through its center)
  neko::PhysicsWorld world{{.gravity = Vec2f::Zero()}};
  const auto plank = world.AddBody();
  const auto plank_collider = world.AddOrientedBoxCollider(plank);
  world.collider(plank_collider).is_trigger = false;
  world.collider(plank_collider).restitution = Scalar{0};
  world.collider(plank_collider).friction = Scalar{0};
  world.oriented_box(world.collider(plank_collider).shape_index).half_size = {
      Scalar{2}, Ratio(1, 4)};
  const auto ball = world.AddBody();
  world.body(ball).position = {Ratio(3, 2), Ratio(1, 2)};
  world.body(ball).velocity = {Scalar{0}, Scalar{-3}};
  const auto ball_collider = world.AddCircleCollider(ball);
  world.collider(ball_collider).is_trigger = false;
  world.circle(world.collider(ball_collider).shape_index).radius = Ratio(3, 10);
  world.Step();
  // Pushed down on its right: the plank turns clockwise
  EXPECT_LT(world.body(plank).angular_velocity, Scalar{0});
  EXPECT_LT(world.body(plank).velocity.y(), Scalar{0});
  EXPECT_EQ(world.body(ball).angular_velocity, Scalar{0});
}

TEST(OffCenterImpulse, TiltedBoxDroppedOnTheGroundStartsTurningFlat) {
  // With gravity: a box dropped tilted lands on its low corner and starts
  // turning toward flat
  neko::PhysicsWorld world;
  const auto ground = world.AddBody();
  world.body(ground).type = neko::BodyType::kStatic;
  world.body(ground).position = {Scalar{0}, Scalar{-1}};
  const auto ground_collider = world.AddAabbCollider(ground);
  world.collider(ground_collider).is_trigger = false;
  world.aabb(world.collider(ground_collider).shape_index).half_size = {
      Scalar{10}, Scalar{1}};
  const auto box = world.AddBody();
  world.body(box).position = {Scalar{0}, Scalar{2}};
  world.body(box).rotation.set_angle(Radian{Degrees(25)});
  const auto box_collider = world.AddOrientedBoxCollider(box);
  world.collider(box_collider).is_trigger = false;
  world.collider(box_collider).restitution = Scalar{0};
  world.oriented_box(world.collider(box_collider).shape_index).half_size =
      kBoxHalfSize;
  int step = 0;
  while (world.touching_pairs().empty() && step < 120) {
    world.Step();
    ++step;
  }
  ASSERT_LT(step, 120);
  EXPECT_LT(world.body(box).angular_velocity, Scalar{0});
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
