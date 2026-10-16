// Oriented boxes (M5 stage 2): SAT against boxes, circles and AABBs, the
// contact normal convention, the deepest-corner contact point, the tight
// broadphase box, inertia, checksums and rollback.
#include <type_traits>
#include <vector>

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

Scalar QuarterTurn() { return neko::Pi<Scalar>() / Scalar{2}; }
Scalar EighthTurn() { return neko::Pi<Scalar>() / Scalar{4}; }

// static_cast<float> would be a useless cast when Scalar is float
template <typename T>
float ToFloat(T value) {
  if constexpr (std::is_same_v<T, float>) {
    return value;
  } else {
    return static_cast<float>(value);
  }
}

void ExpectNear(Scalar actual, Scalar expected, Scalar tolerance) {
  EXPECT_LT(neko::Abs(actual - expected), tolerance)
      << ToFloat(actual) << " vs " << ToFloat(expected);
}

void ExpectNear(Vec2f actual, Vec2f expected, Scalar tolerance) {
  ExpectNear(actual.x(), expected.x(), tolerance);
  ExpectNear(actual.y(), expected.y(), tolerance);
}

const Scalar kTolerance = Ratio(1, 1000);

// A world without gravity and helpers to place solid shapes
struct Fixture {
  BodyIndex AddBody(Vec2f position, Scalar angle = Scalar{0}) {
    const auto index = world.AddBody();
    world.body(index).position = position;
    world.body(index).rotation.set_angle(Radian{angle});
    return index;
  }
  ColliderIndex AddOrientedBox(BodyIndex body, Vec2f half_size) {
    const auto index = world.AddOrientedBoxCollider(body);
    world.collider(index).is_trigger = false;
    world.oriented_box(world.collider(index).shape_index).half_size = half_size;
    return index;
  }
  ColliderIndex AddAabb(BodyIndex body, Vec2f half_size) {
    const auto index = world.AddAabbCollider(body);
    world.collider(index).is_trigger = false;
    world.aabb(world.collider(index).shape_index).half_size = half_size;
    return index;
  }
  ColliderIndex AddCircle(BodyIndex body, Scalar radius) {
    const auto index = world.AddCircleCollider(body);
    world.collider(index).is_trigger = false;
    world.circle(world.collider(index).shape_index).radius = radius;
    return index;
  }
  bool Detect(ColliderIndex collider1, ColliderIndex collider2,
              neko::Contact* contact) {
    auto& c1 = world.collider(collider1);
    auto& c2 = world.collider(collider2);
    return world.DetectContact(world.body(c1.body_index), c1,
                               world.body(c2.body_index), c2, contact);
  }

  neko::PhysicsWorld world{{.gravity = Vec2f::Zero()}};
};

}  // namespace

TEST(OrientedBox, UnturnedBoxMatchesTheAabbContact) {
  // A 2 x 2 box sinking 0.25 into a wide ground, as an oriented box and as an
  // AABB: same normal and penetration
  Fixture f;
  const auto ground = f.AddBody({Scalar{0}, Scalar{-1}});
  const auto ground_box = f.AddAabb(ground, {Scalar{10}, Scalar{1}});
  const auto oriented = f.AddOrientedBox(f.AddBody({Scalar{0}, Ratio(3, 4)}),
                                         {Scalar{1}, Scalar{1}});
  const auto aabb =
      f.AddAabb(f.AddBody({Scalar{0}, Ratio(3, 4)}), {Scalar{1}, Scalar{1}});
  neko::Contact oriented_contact{};
  neko::Contact aabb_contact{};
  ASSERT_TRUE(f.Detect(oriented, ground_box, &oriented_contact));
  ASSERT_TRUE(f.Detect(aabb, ground_box, &aabb_contact));
  EXPECT_EQ(oriented_contact.contact_normal, aabb_contact.contact_normal);
  EXPECT_EQ(oriented_contact.contact_normal, Vec2f::Up());
  EXPECT_EQ(oriented_contact.penetration, aabb_contact.penetration);
  EXPECT_EQ(oriented_contact.penetration, Ratio(1, 4));
}

TEST(OrientedBox, DiamondOnGroundTouchesWithItsBottomCorner) {
  // A 2 x 2 square turned 45 degrees: its bottom corner is sqrt(2) below its
  // center. Center at 1.3 above the ground: the corner sinks by
  // sqrt(2) - 1.3, about 0.114
  Fixture f;
  const auto ground =
      f.AddAabb(f.AddBody({Scalar{0}, Scalar{-1}}), {Scalar{10}, Scalar{1}});
  const auto diamond =
      f.AddOrientedBox(f.AddBody({Scalar{0}, Ratio(13, 10)}, EighthTurn()),
                       {Scalar{1}, Scalar{1}});
  neko::Contact contact{};
  ASSERT_TRUE(f.Detect(diamond, ground, &contact));
  // The ground's face (body 2) is the reference: the normal points up, from
  // the ground toward the diamond
  ExpectNear(contact.contact_normal, Vec2f::Up(), kTolerance);
  ExpectNear(contact.penetration, Ratio(1414, 1000) - Ratio(13, 10),
             Ratio(3, 1000));
  ExpectNear(contact.contact_point,
             {Scalar{0}, Ratio(13, 10) - Ratio(1414, 1000)}, Ratio(3, 1000));
}

TEST(OrientedBox, SeparatedDiamondsWhoseBoundingBoxesOverlap) {
  // Two 45-degree diamonds side by side, corner to corner along a diagonal
  // gap: their AABBs overlap, their shapes don't (SAT finds a separating
  // face normal)
  Fixture f;
  const auto a = f.AddOrientedBox(
      f.AddBody({Scalar{0}, Scalar{0}}, EighthTurn()), {Scalar{1}, Scalar{1}});
  const auto b =
      f.AddOrientedBox(f.AddBody({Ratio(23, 10), Ratio(23, 10)}, EighthTurn()),
                       {Scalar{1}, Scalar{1}});
  neko::Contact contact{};
  EXPECT_FALSE(f.Detect(a, b, &contact));
  f.world.ResolveBroadphase();
  EXPECT_EQ(f.world.BroadphasePairCount(), 1u)
      << "the broadphase still pairs them";
}

TEST(OrientedBox, NormalPointsFromBody2TowardBody1) {
  Fixture f;
  const auto upper =
      f.AddOrientedBox(f.AddBody({Ratio(1, 10), Ratio(15, 10)}, Ratio(1, 10)),
                       {Scalar{1}, Scalar{1}});
  const auto lower = f.AddOrientedBox(
      f.AddBody({Scalar{0}, Scalar{0}}, Ratio(-1, 10)), {Scalar{1}, Scalar{1}});
  neko::Contact contact{};
  ASSERT_TRUE(f.Detect(upper, lower, &contact));
  EXPECT_GT(contact.contact_normal.y(), Ratio(9, 10));
  ASSERT_TRUE(f.Detect(lower, upper, &contact));
  EXPECT_LT(contact.contact_normal.y(), -Ratio(9, 10));
  EXPECT_GT(contact.penetration, Scalar{0});
}

TEST(OrientedBox, TouchingCountsAsOverlapping) {
  Fixture f;
  const auto a = f.AddOrientedBox(f.AddBody({Scalar{0}, Scalar{2}}),
                                  {Scalar{1}, Scalar{1}});
  const auto b = f.AddOrientedBox(f.AddBody({Scalar{0}, Scalar{0}}),
                                  {Scalar{1}, Scalar{1}});
  neko::Contact contact{};
  EXPECT_TRUE(f.Detect(a, b, &contact));
  EXPECT_EQ(contact.penetration, Scalar{0});
}

TEST(OrientedBox, EqualOverlapsPickTheFirstBodysFace) {
  // Two identical unturned boxes overlapping diagonally by 0.5 on both axes:
  // four equal overlaps. The fixed order (body 1's x axis first, strictly
  // smaller to replace it) picks body 1's x axis, whichever body is first.
  Fixture f;
  const auto a = f.AddOrientedBox(f.AddBody({Ratio(3, 2), Ratio(3, 2)}),
                                  {Scalar{1}, Scalar{1}});
  const auto b = f.AddOrientedBox(f.AddBody({Scalar{0}, Scalar{0}}),
                                  {Scalar{1}, Scalar{1}});
  neko::Contact contact{};
  ASSERT_TRUE(f.Detect(a, b, &contact));
  EXPECT_EQ(contact.contact_normal, Vec2f::Right());
  ASSERT_TRUE(f.Detect(b, a, &contact));
  EXPECT_EQ(contact.contact_normal, Vec2f::Left());
}

TEST(OrientedBox, CircleOnATiltedFaceGetsTheFaceNormal) {
  // A ramp turned 30 degrees, a circle resting slightly into its top face
  Fixture f;
  const Scalar angle = neko::Pi<Scalar>() / Scalar{6};
  const auto ramp_body = f.AddBody({Scalar{0}, Scalar{0}}, angle);
  const auto ramp = f.AddOrientedBox(ramp_body, {Scalar{4}, Ratio(1, 2)});
  const Vec2f face_normal = f.world.body(ramp_body).rotation.y_axis();
  const Vec2f center = face_normal * (Ratio(1, 2) + Ratio(4, 10));
  const auto ball = f.AddCircle(f.AddBody(center), Ratio(1, 2));
  neko::Contact contact{};
  ASSERT_TRUE(f.Detect(ball, ramp, &contact));
  // The ramp is body 1 after the swap: the normal points from the ball
  // toward the ramp
  EXPECT_EQ(contact.bodies[0].collider, &f.world.collider(ramp));
  ExpectNear(contact.contact_normal, -face_normal, kTolerance);
  ExpectNear(contact.penetration, Ratio(1, 10), kTolerance);
  ExpectNear(contact.contact_point, face_normal * Ratio(1, 2), kTolerance);
}

TEST(OrientedBox, CircleCenterInsidePushesOutThroughTheNearestFace) {
  Fixture f;
  const auto box_body = f.AddBody({Scalar{0}, Scalar{0}}, QuarterTurn());
  // Local 2 x 0.5, turned a quarter: 1 wide and 4 tall in the world
  const auto box = f.AddOrientedBox(box_body, {Scalar{2}, Ratio(1, 2)});
  const auto ball =
      f.AddCircle(f.AddBody({Ratio(3, 10), Scalar{1}}), Ratio(1, 4));
  neko::Contact contact{};
  ASSERT_TRUE(f.Detect(box, ball, &contact));
  // Nearest face: the world right side (local -y), 0.2 away
  ExpectNear(contact.contact_normal, Vec2f::Left(), kTolerance);
  ExpectNear(contact.penetration, Ratio(45, 100), kTolerance);
}

TEST(OrientedBox, BroadphaseBoxIsTheTightTurnedExtent) {
  Fixture f;
  // A 4 x 1 plank turned 90 degrees is 1 wide and 4 tall: a circle 1 m to its
  // right is outside its broadphase box, one 1.5 m above isn't
  const auto plank_body = f.AddBody({Scalar{0}, Scalar{0}}, QuarterTurn());
  f.AddOrientedBox(plank_body, {Scalar{2}, Ratio(1, 2)});
  const auto side = f.AddBody({Ratio(11, 10), Scalar{0}});
  f.AddCircle(side, Ratio(1, 2));
  f.world.ResolveBroadphase();
  EXPECT_EQ(f.world.BroadphasePairCount(), 0u);
  f.world.body(side).position = {Scalar{0}, Ratio(23, 10)};
  f.world.ResolveBroadphase();
  EXPECT_EQ(f.world.BroadphasePairCount(), 1u);
}

TEST(OrientedBox, InertiaIsTheSameAsABox) {
  Fixture f;
  const auto oriented = f.AddBody({});
  f.AddOrientedBox(oriented, {Scalar{1}, Ratio(1, 2)});
  const auto aabb = f.AddBody({});
  f.AddAabb(aabb, {Scalar{1}, Ratio(1, 2)});
  EXPECT_EQ(f.world.ComputeInverseInertia(oriented),
            f.world.ComputeInverseInertia(aabb));
}

TEST(OrientedBox, RemoveFreesTheSlotAndChecksumCoversTheSize) {
  Fixture f;
  const auto body = f.AddBody({});
  const auto collider = f.AddOrientedBox(body, {Scalar{1}, Scalar{1}});
  const auto checksum = f.world.GenerateChecksum();
  f.world.oriented_box(f.world.collider(collider).shape_index).half_size = {
      Scalar{1}, Ratio(1, 2)};
  EXPECT_NE(f.world.GenerateChecksum(), checksum);
  const auto shape = f.world.collider(collider).shape_index;
  f.world.RemoveCollider(collider);
  EXPECT_EQ(f.world.collider(collider).type, neko::ShapeType::kNone);
  // The freed slots come back
  const auto again = f.AddOrientedBox(body, {Scalar{1}, Scalar{1}});
  EXPECT_EQ(again, collider);
  EXPECT_EQ(f.world.collider(again).shape_index, shape);
}

TEST(OrientedBox, TiltedBoxesFallingOnARampReplayIdentically) {
  const auto build = [](neko::PhysicsWorld& world) {
    const auto ramp = world.AddBody();
    world.body(ramp).type = neko::BodyType::kStatic;
    world.body(ramp).rotation.set_angle(Radian{Ratio(-1, 5)});
    const auto ramp_collider = world.AddOrientedBoxCollider(ramp);
    world.collider(ramp_collider).is_trigger = false;
    world.collider(ramp_collider).restitution = Ratio(1, 4);
    world.oriented_box(world.collider(ramp_collider).shape_index).half_size = {
        Scalar{8}, Ratio(1, 2)};
    for (int i = 0; i < 8; ++i) {
      const auto body = world.AddBody();
      world.body(body).position = {Ratio(-6 + 2 * i, 1), Ratio(3 + i % 3, 1)};
      world.body(body).angular_velocity = Ratio(i % 5 - 2, 2);
      const auto collider = i % 2 == 0 ? world.AddOrientedBoxCollider(body)
                                       : world.AddCircleCollider(body);
      world.collider(collider).is_trigger = false;
      world.collider(collider).restitution = Ratio(1, 5);
      if (i % 2 == 0) {
        world.oriented_box(world.collider(collider).shape_index).half_size = {
            Ratio(1, 2), Ratio(1, 4)};
      } else {
        world.circle(world.collider(collider).shape_index).radius = Ratio(1, 3);
      }
    }
  };
  neko::PhysicsWorld world;
  build(world);
  for (int step = 0; step < 60; ++step) {
    world.Step();
  }
  EXPECT_FALSE(world.touching_pairs().empty());
  neko::PhysicsWorld snapshot;
  snapshot.CopyFrom(world);
  std::vector<std::uint32_t> checksums;
  for (int step = 0; step < 120; ++step) {
    world.Step();
    checksums.push_back(world.GenerateChecksum());
  }
  world.CopyFrom(snapshot);
  for (std::size_t step = 0; step < checksums.size(); ++step) {
    world.Step();
    ASSERT_EQ(world.GenerateChecksum(), checksums[step]) << step;
  }
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
