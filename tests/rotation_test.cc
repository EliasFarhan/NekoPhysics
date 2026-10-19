// Angular motion (M5 stage 1): Rotation, inertia from colliders, torque
// integration, forces and impulses at a point, and collider offsets that turn
// with their body.
#include "neko/math/rotation.h"

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
  // In 64 bits: Scalar{den} overflows Fixed16 from 32768 up
  return neko::ScalarTraits<Scalar>::FromRatio(num, den);
}

Scalar Pi() { return neko::Pi<Scalar>(); }

// ScalarTraits picks the conversion (none for float: it would be a useless
// cast)
template <typename T>
float ToFloat(T value) {
  return neko::ScalarTraits<T>::ToFloat(value);
}

// Close enough for values computed with table sines (5e-5) and 1/65536 steps
void ExpectNear(Scalar actual, Scalar expected, Scalar tolerance) {
  EXPECT_LT(neko::Abs(actual - expected), tolerance)
      << ToFloat(actual) << " vs " << ToFloat(expected);
}

void ExpectNear(Vec2f actual, Vec2f expected, Scalar tolerance) {
  ExpectNear(actual.x(), expected.x(), tolerance);
  ExpectNear(actual.y(), expected.y(), tolerance);
}

// A world without gravity, with helpers to build bodies of known inertia
struct Fixture {
  BodyIndex AddBody(Scalar mass, Vec2f position = {}) {
    const auto index = world.AddBody();
    world.body(index).position = position;
    world.body(index).inverse_mass = Scalar{1} / mass;
    return index;
  }
  ColliderIndex AddCircle(BodyIndex body, Scalar radius, Vec2f offset = {},
                          bool is_trigger = false) {
    const auto index = world.AddCircleCollider(body);
    world.collider(index).is_trigger = is_trigger;
    world.collider(index).offset = offset;
    world.circle(world.collider(index).shape_index).radius = radius;
    return index;
  }
  ColliderIndex AddBox(BodyIndex body, Vec2f half_size, Vec2f offset = {}) {
    const auto index = world.AddAabbCollider(body);
    world.collider(index).is_trigger = false;
    world.collider(index).offset = offset;
    world.aabb(world.collider(index).shape_index).half_size = half_size;
    return index;
  }
  // Moment of inertia in kg m^2 as the world computes it
  Scalar Inertia(BodyIndex body) const {
    return Scalar{1} / world.ComputeInverseInertia(body);
  }

  neko::PhysicsWorld world{{.gravity = Vec2f::Zero()}};
};

}  // namespace

TEST(Vec2Cross, SignFollowsTheTurnFromFirstToSecond) {
  const Vec2f lever{Scalar{2}, Scalar{0}};
  const Vec2f push{Scalar{0}, Scalar{3}};
  EXPECT_EQ(Vec2f::Cross(lever, push), Scalar{6});
  EXPECT_EQ(Vec2f::Cross(push, lever), Scalar{-6});
  EXPECT_EQ(Vec2f::Cross(lever, lever * Scalar{3}), Scalar{0});
}

TEST(WrapAngle, KeepsOneTurnFromMinusPiIncludedToPiExcluded) {
  EXPECT_EQ(neko::WrapAngle(Radian{Pi()}).value(), -Pi());
  EXPECT_EQ(neko::WrapAngle(Radian{-Pi()}).value(), -Pi());
  const Radian inside{Ratio(3, 2)};
  EXPECT_EQ(neko::WrapAngle(inside).value(), inside.value());
  ExpectNear(neko::WrapAngle(Radian{Pi() * Ratio(3, 2)}).value(),
             -Pi() / Scalar{2}, Ratio(1, 10000));
  ExpectNear(neko::WrapAngle(Radian{Pi() * Scalar{-7}}).value(), -Pi(),
             Ratio(1, 10000));
}

TEST(Rotation, DefaultLeavesVectorsUnchangedBitForBit) {
  const neko::Rotation rotation;
  EXPECT_EQ(rotation.angle().value(), Scalar{0});
  for (const Vec2f v : {Vec2f{Ratio(-7, 3), Ratio(5, 7)},
                        Vec2f{Ratio(1, 65536), Scalar{-1000}}}) {
    EXPECT_EQ(rotation.Rotate(v), v);
    EXPECT_EQ(rotation.InverseRotate(v), v);
  }
  // Setting the angle 0 explicitly gives the same cache (table sin 0, cos 1)
  neko::Rotation zero;
  zero.set_angle(Radian{});
  EXPECT_EQ(zero.sin(), Scalar{0});
  EXPECT_EQ(zero.cos(), Scalar{1});
}

TEST(Rotation, QuarterTurnAndInverse) {
  const neko::Rotation rotation{Radian{Pi() / Scalar{2}}};
  const auto tolerance = Ratio(1, 10000);
  ExpectNear(rotation.x_axis(), Vec2f::Up(), tolerance);
  ExpectNear(rotation.y_axis(), Vec2f::Left(), tolerance);
  const Vec2f v{Ratio(3, 2), Ratio(-1, 4)};
  ExpectNear(rotation.Rotate(v), Vec2f{Ratio(1, 4), Ratio(3, 2)}, tolerance);
  ExpectNear(rotation.InverseRotate(rotation.Rotate(v)), v, tolerance);
}

TEST(Inertia, CircleIsHalfMassTimesRadiusSquared) {
  Fixture f;
  const auto body = f.AddBody(Scalar{2});
  f.AddCircle(body, Ratio(1, 2));
  // 2 * 0.25 / 2
  ExpectNear(f.Inertia(body), Ratio(1, 4), Ratio(1, 1000));
}

TEST(Inertia, BoxIsMassTimesWidthSquaredPlusHeightSquaredOverTwelve) {
  Fixture f;
  const auto body = f.AddBody(Scalar{3});
  f.AddBox(body, {Scalar{1}, Ratio(1, 2)});
  // 3 * (2^2 + 1^2) / 12
  ExpectNear(f.Inertia(body), Ratio(5, 4), Ratio(1, 1000));
}

TEST(Inertia, OffsetAddsMassTimesDistanceSquared) {
  Fixture f;
  const auto body = f.AddBody(Scalar{1});
  f.AddCircle(body, Ratio(1, 2), {Scalar{2}, Scalar{0}});
  // Parallel axis: 0.125 + 1 * 2^2
  ExpectNear(f.Inertia(body), Ratio(33, 8), Ratio(1, 1000));
}

TEST(Inertia, MassIsSpreadByAreaOverSolidCollidersOnly) {
  Fixture f;
  // Dumbbell: two equal circles, 1 kg each, 1 m on each side
  const auto body = f.AddBody(Scalar{2});
  f.AddCircle(body, Ratio(1, 2), {Scalar{-1}, Scalar{0}});
  f.AddCircle(body, Ratio(1, 2), {Scalar{1}, Scalar{0}});
  const Scalar dumbbell = f.Inertia(body);
  ExpectNear(dumbbell, Ratio(9, 4), Ratio(1, 1000));
  // A large trigger (a pickup radius) has no mass
  f.AddCircle(body, Scalar{5}, {}, true);
  EXPECT_EQ(f.Inertia(body), dumbbell);
}

TEST(Inertia, NoSolidShapeOrInfiniteMassMeansNoSpin) {
  Fixture f;
  const auto empty = f.AddBody(Scalar{1});
  EXPECT_EQ(f.world.ComputeInverseInertia(empty), Scalar{0});
  const auto trigger_only = f.AddBody(Scalar{1});
  f.AddCircle(trigger_only, Scalar{1}, {}, true);
  EXPECT_EQ(f.world.ComputeInverseInertia(trigger_only), Scalar{0});
  const auto zero_size = f.AddBody(Scalar{1});
  f.AddCircle(zero_size, Scalar{0}, {Scalar{1}, Scalar{0}});
  EXPECT_EQ(f.world.ComputeInverseInertia(zero_size), Scalar{0});
  const auto heavy = f.AddBody(Scalar{1});
  f.world.body(heavy).inverse_mass = Scalar{0};
  f.AddCircle(heavy, Scalar{1});
  EXPECT_EQ(f.world.ComputeInverseInertia(heavy), Scalar{0});
}

TEST(Inertia, StepComputesItForDynamicBodiesFromCollidersMode) {
  Fixture f;
  const auto computed = f.AddBody(Scalar{2});
  f.AddBox(computed, {Scalar{1}, Ratio(1, 2)}, {Ratio(1, 3), Scalar{0}});
  f.AddCircle(computed, Ratio(1, 3), {Scalar{-1}, Ratio(1, 2)});
  const auto overridden = f.AddBody(Scalar{1});
  f.AddCircle(overridden, Scalar{1});
  f.world.body(overridden).inertia_mode = neko::InertiaMode::kOverride;
  f.world.body(overridden).inverse_inertia = Scalar{7};
  const auto fixed = f.AddBody(Scalar{1});
  f.AddCircle(fixed, Scalar{1});
  f.world.body(fixed).inertia_mode = neko::InertiaMode::kFixedRotation;
  f.world.body(fixed).inverse_inertia = Scalar{7};
  EXPECT_EQ(f.world.body(computed).inverse_inertia, Scalar{0})
      << "0 until the first step";

  f.world.Step();
  // The batched pass in Step and ComputeInverseInertia give the same bits
  EXPECT_EQ(f.world.body(computed).inverse_inertia,
            f.world.ComputeInverseInertia(computed));
  EXPECT_EQ(f.world.body(overridden).inverse_inertia, Scalar{7});
  EXPECT_EQ(f.world.body(fixed).inverse_inertia, Scalar{0});
}

TEST(AngularMotion, TorqueIsIntegratedWithSemiImplicitEuler) {
  Fixture f;
  const auto body = f.AddBody(Scalar{2});
  f.AddCircle(body, Ratio(1, 2));
  const Scalar torque = Ratio(3, 10);
  const Scalar dt = f.world.dt();
  f.world.Step();  // computes the inertia
  const Scalar inverse_inertia = f.world.body(body).inverse_inertia;

  Scalar angular_velocity{};
  Radian angle{};
  for (int step = 0; step < 240; ++step) {
    f.world.body(body).torque = torque;
    f.world.Step();
    EXPECT_EQ(f.world.body(body).torque, Scalar{0}) << "cleared like force";
    // Same operations in the same order: same bits
    angular_velocity += torque * inverse_inertia * dt;
    angle = neko::WrapAngle(angle + Radian{angular_velocity * dt});
    ASSERT_EQ(f.world.body(body).angular_velocity, angular_velocity) << step;
    ASSERT_EQ(f.world.body(body).rotation.angle().value(), angle.value())
        << step;
  }
  EXPECT_EQ(f.world.body(body).position, Vec2f::Zero()) << "pure torque";
}

TEST(AngularMotion, AngleStaysWrappedWhileSpinning) {
  Fixture f;
  const auto body = f.AddBody(Scalar{1});
  f.AddCircle(body, Scalar{1});
  f.world.body(body).angular_velocity = Scalar{20};
  Scalar previous = f.world.body(body).rotation.angle().value();
  int wraps = 0;
  for (int step = 0; step < 600; ++step) {
    f.world.Step();
    const Scalar angle = f.world.body(body).rotation.angle().value();
    ASSERT_GE(angle, -Pi());
    ASSERT_LT(angle, Pi());
    wraps += angle < previous ? 1 : 0;
    previous = angle;
  }
  // 20 rad/s for 10 s: 200 rad, about 31.8 turns
  EXPECT_EQ(wraps, 32);
}

TEST(AngularMotion, KinematicTurnsAtItsVelocityStaticDoesNot) {
  Fixture f;
  const auto kinematic = f.AddBody(Scalar{1});
  f.world.body(kinematic).type = neko::BodyType::kKinematic;
  f.world.body(kinematic).angular_velocity = Scalar{1};
  const auto fixed = f.AddBody(Scalar{1});
  f.world.body(fixed).type = neko::BodyType::kStatic;
  f.world.body(fixed).angular_velocity = Scalar{1};
  for (int step = 0; step < 60; ++step) {
    f.world.body(kinematic).torque = Scalar{100};
    f.world.Step();
  }
  ExpectNear(f.world.body(kinematic).rotation.angle().value(), Scalar{1},
             Ratio(1, 100));
  EXPECT_EQ(f.world.body(kinematic).angular_velocity, Scalar{1});
  EXPECT_EQ(f.world.body(fixed).rotation.angle().value(), Scalar{0});
}

TEST(AngularMotion, FixedRotationIgnoresTorqueAndImpulses) {
  Fixture f;
  const auto body = f.AddBody(Scalar{1});
  f.AddCircle(body, Scalar{1});
  f.world.body(body).inertia_mode = neko::InertiaMode::kFixedRotation;
  f.world.ApplyImpulseAtPoint(body, {Scalar{0}, Scalar{1}},
                              {Scalar{1}, Scalar{0}});
  for (int step = 0; step < 30; ++step) {
    f.world.body(body).torque = Scalar{5};
    f.world.Step();
  }
  EXPECT_EQ(f.world.body(body).angular_velocity, Scalar{0});
  EXPECT_EQ(f.world.body(body).rotation.angle().value(), Scalar{0});
  EXPECT_NE(f.world.body(body).velocity, Vec2f::Zero()) << "still pushed";
}

TEST(AngularMotion, BodiesThatDontTurnKeepTheDefaultRotation) {
  neko::PhysicsWorld world;  // with gravity
  const auto body = world.AddBody();
  const auto collider = world.AddAabbCollider(body);
  world.aabb(world.collider(collider).shape_index).half_size = Vec2f::One();
  for (int step = 0; step < 60; ++step) {
    world.Step();
  }
  EXPECT_EQ(world.body(body).rotation, neko::Rotation{});
  EXPECT_EQ(world.body(body).rotation.cos(), Scalar{1});
}

TEST(ForceAtPoint, AddsTheForceAndTheTorqueOfItsLeverArm) {
  Fixture f;
  const Vec2f position{Scalar{3}, Scalar{-2}};
  const auto body = f.AddBody(Scalar{1}, position);
  // Up at 0.5 m to the right: counter-clockwise, 0.5 * 10
  f.world.ApplyForceAtPoint(body, {Scalar{0}, Scalar{10}},
                            position + Vec2f{Ratio(1, 2), Scalar{0}});
  EXPECT_EQ(f.world.body(body).force, (Vec2f{Scalar{0}, Scalar{10}}));
  EXPECT_EQ(f.world.body(body).torque, Scalar{5});
  // Through the center: no torque
  f.world.ApplyForceAtPoint(body, {Scalar{4}, Scalar{0}}, position);
  EXPECT_EQ(f.world.body(body).torque, Scalar{5});
}

TEST(ForceAtPoint, ACoupleSpinsInPlace) {
  Fixture f;
  const auto body = f.AddBody(Scalar{1});
  f.AddBox(body, {Scalar{1}, Scalar{1}});
  for (int step = 0; step < 60; ++step) {
    f.world.ApplyForceAtPoint(body, {Scalar{0}, Scalar{2}},
                              {Scalar{1}, Scalar{0}});
    f.world.ApplyForceAtPoint(body, {Scalar{0}, Scalar{-2}},
                              {Scalar{-1}, Scalar{0}});
    EXPECT_EQ(f.world.body(body).force, Vec2f::Zero());
    EXPECT_EQ(f.world.body(body).torque, Scalar{4});
    f.world.Step();
  }
  EXPECT_EQ(f.world.body(body).position, Vec2f::Zero());
  EXPECT_GT(f.world.body(body).angular_velocity, Scalar{0});
}

TEST(ImpulseAtPoint, ChangesBothVelocitiesAtOnceEvenBeforeTheFirstStep) {
  Fixture f;
  const auto body = f.AddBody(Scalar{1});
  f.AddCircle(body, Scalar{1});  // inertia 0.5
  f.world.ApplyImpulseAtPoint(body, {Scalar{0}, Scalar{1}},
                              {Scalar{1}, Scalar{0}});
  EXPECT_EQ(f.world.body(body).velocity, (Vec2f{Scalar{0}, Scalar{1}}));
  // 1 * 1 / 0.5
  ExpectNear(f.world.body(body).angular_velocity, Scalar{2}, Ratio(1, 1000));

  const auto kinematic = f.AddBody(Scalar{1});
  f.world.body(kinematic).type = neko::BodyType::kKinematic;
  f.world.ApplyImpulseAtPoint(kinematic, {Scalar{0}, Scalar{1}},
                              {Scalar{1}, Scalar{0}});
  EXPECT_EQ(f.world.body(kinematic).velocity, Vec2f::Zero());
  EXPECT_EQ(f.world.body(kinematic).angular_velocity, Scalar{0});
}

TEST(ColliderOffset, TurnsWithTheBody) {
  Fixture f;
  const Vec2f position{Scalar{1}, Scalar{1}};
  const auto body = f.AddBody(Scalar{1}, position);
  const auto circle = f.AddCircle(body, Ratio(1, 2), {Scalar{2}, Scalar{0}});
  EXPECT_EQ(f.world.ColliderCenter(circle),
            (position + Vec2f{Scalar{2}, Scalar{0}}));
  f.world.body(body).rotation.set_angle(Radian{Pi() / Scalar{2}});
  ExpectNear(f.world.ColliderCenter(circle),
             position + Vec2f{Scalar{0}, Scalar{2}}, Ratio(1, 1000));

  // The narrowphase uses the turned offset: a circle above the body touches
  // it, one at the unturned offset doesn't
  const auto above =
      f.AddBody(Scalar{1}, position + Vec2f{Scalar{0}, Scalar{3}});
  const auto above_circle = f.AddCircle(above, Ratio(3, 4));
  const auto right =
      f.AddBody(Scalar{1}, position + Vec2f{Scalar{3}, Scalar{0}});
  const auto right_circle = f.AddCircle(right, Ratio(3, 4));
  neko::Contact contact{};
  EXPECT_TRUE(f.world.DetectContact(
      f.world.body(above), f.world.collider(above_circle), f.world.body(body),
      f.world.collider(circle), &contact));
  EXPECT_FALSE(f.world.DetectContact(
      f.world.body(right), f.world.collider(right_circle), f.world.body(body),
      f.world.collider(circle), &contact));
}

TEST(ColliderOffset, AabbStaysAxisAlignedAtTheTurnedOffset) {
  Fixture f;
  const auto body = f.AddBody(Scalar{1});
  // A wide flat box 2 m to the right; the body turns a quarter turn
  const auto box =
      f.AddBox(body, {Scalar{1}, Ratio(1, 4)}, {Scalar{2}, Scalar{0}});
  f.world.body(body).rotation.set_angle(Radian{Pi() / Scalar{2}});
  // Its center moved above the body, but it is still wide and flat: a point
  // 0.9 m to the side of the new center is inside it
  const Vec2f center = f.world.ColliderCenter(box);
  const auto probe =
      f.AddBody(Scalar{1}, center + Vec2f{Ratio(9, 10), Scalar{0}});
  const auto probe_circle = f.AddCircle(probe, Ratio(1, 20));
  neko::Contact contact{};
  EXPECT_TRUE(f.world.DetectContact(f.world.body(body), f.world.collider(box),
                                    f.world.body(probe),
                                    f.world.collider(probe_circle), &contact));
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
