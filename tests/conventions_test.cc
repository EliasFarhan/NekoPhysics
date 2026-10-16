// The conventions documented in the namespace comment of
// include/neko/physics/physics.h ("Conventions"), checked so the
// documentation can't drift from the code.
#include <gtest/gtest.h>

#include "neko/math/const.h"
#include "neko/math/func.h"
#include "neko/physics/physics.h"

namespace {

constexpr neko::Scalar Ratio(int num, int den) {
  return static_cast<neko::Scalar>(num) / static_cast<neko::Scalar>(den);
}

// Two solid circles of radius 1 on dynamic bodies, without gravity
struct TwoCircles {
  TwoCircles(neko::Vec2f position1, neko::Vec2f position2) {
    for (const auto& position : {position1, position2}) {
      const auto body = world.AddBody();
      world.body(body).position = position;
      const auto collider = world.AddCircleCollider(body);
      world.collider(collider).is_trigger = false;
      world.circle(world.collider(collider).shape_index).radius =
          neko::Scalar{1};
    }
  }
  bool Detect(neko::Contact* contact) {
    return world.DetectContact(world.body(neko::BodyIndex{0}),
                               world.collider(neko::ColliderIndex{0}),
                               world.body(neko::BodyIndex{1}),
                               world.collider(neko::ColliderIndex{1}), contact);
  }
  neko::PhysicsWorld world{{.gravity = neko::Vec2f::Zero()}};
};

}  // namespace

TEST(Conventions, WorldIsYUpWithGravityDown) {
  const neko::PhysicsWorldConfig config;
  EXPECT_EQ(config.gravity.x(), neko::Scalar{0});
  EXPECT_LT(config.gravity.y(), neko::Scalar{0});
  EXPECT_EQ(neko::Vec2f::Up(), (neko::Vec2f{neko::Scalar{0}, neko::Scalar{1}}));
}

TEST(Conventions, PositiveAngleIsCounterClockwise) {
  // A quarter turn takes +x to +y and +y to -x
  const auto quarter_turn = neko::Pi<neko::Scalar>() / neko::Scalar{2};
  const auto right = neko::Vec2f::Right().Rotate(quarter_turn);
  const auto up = neko::Vec2f::Up().Rotate(quarter_turn);
  const auto tolerance = Ratio(1, 1000);
  EXPECT_LT(neko::Abs(right.x()), tolerance);
  EXPECT_LT(neko::Abs(right.y() - neko::Scalar{1}), tolerance);
  EXPECT_LT(neko::Abs(up.x() + neko::Scalar{1}), tolerance);
  EXPECT_LT(neko::Abs(up.y()), tolerance);
}

TEST(Conventions, NormalPointsFromBody2TowardBody1) {
  // Body 1 above body 2, overlapping by 0.5
  TwoCircles circles{{neko::Scalar{0}, Ratio(3, 2)},
                     {neko::Scalar{0}, neko::Scalar{0}}};
  neko::Contact contact{};
  ASSERT_TRUE(circles.Detect(&contact));
  // Unit length up to rounding: the normal is midline * (1 / distance)
  EXPECT_EQ(contact.contact_normal.x(), neko::Scalar{0});
  EXPECT_LT(neko::Abs(contact.contact_normal.y() - neko::Scalar{1}),
            Ratio(1, 10000));
  EXPECT_EQ(contact.penetration, Ratio(1, 2));
}

TEST(Conventions, TouchingCountsAsOverlapping) {
  // Centers exactly two radii apart: touching, zero penetration
  TwoCircles circles{{neko::Scalar{2}, neko::Scalar{0}},
                     {neko::Scalar{0}, neko::Scalar{0}}};
  neko::Contact contact{};
  EXPECT_TRUE(circles.Detect(&contact));
  EXPECT_EQ(contact.penetration, neko::Scalar{0});
  circles.world.Step();
  EXPECT_EQ(circles.world.touching_pairs().size(), 1u);
}

TEST(Conventions, NoSlopPenetrationIsCorrectedInOneStep) {
  // Overlapping by 0.5 at rest: one step pushes them exactly apart, half each
  // (equal masses)
  TwoCircles circles{{neko::Scalar{0}, Ratio(3, 2)},
                     {neko::Scalar{0}, neko::Scalar{0}}};
  circles.world.Step();
  const auto distance = (circles.world.body(neko::BodyIndex{0}).position -
                         circles.world.body(neko::BodyIndex{1}).position)
                            .Length();
  EXPECT_LT(neko::Abs(distance - neko::Scalar{2}), Ratio(1, 10000));
}

// A macro rather than if constexpr: outside a template, the discarded branch
// must still compile, and it uses the Fixed16 raw value
TEST(Conventions, Fixed16SlowestMovingSpeed) {
#ifndef NEKO_SCALAR_FIXED16
  GTEST_SKIP() << "Fixed16 resolution";
#else
  {
    // dt = 1/60 s is stored as 1092 / 65536 s (0.0166626 s, 60 steps are
    // 0.99976 s). A position moves by velocity * dt rounded to 1 / 65536 m:
    // below 31 / 65536 m/s (about 0.47 mm/s) the step rounds to zero
    const neko::PhysicsWorldConfig config;
    EXPECT_EQ(config.dt.underlying_value(), 1092);
    const auto still = neko::Fixed16::FromUnderlyingValue(30);
    const auto moving = neko::Fixed16::FromUnderlyingValue(31);
    EXPECT_EQ(still * config.dt, neko::Scalar{0});
    EXPECT_NE(moving * config.dt, neko::Scalar{0});

    neko::PhysicsWorld world{{.gravity = neko::Vec2f::Zero()}};
    const auto body = world.AddBody();
    world.body(body).velocity = {still, neko::Scalar{0}};
    for (int step = 0; step < 600; ++step) {
      world.Step();
    }
    EXPECT_EQ(world.body(body).position, neko::Vec2f::Zero());
  }
#endif
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
