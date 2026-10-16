// Contact manifolds (M5 stage 3): up to two points for box contacts, found by
// clipping the incident face against the reference face, one point for
// circles.
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

void ExpectNear(Scalar actual, Scalar expected, Scalar tolerance) {
  EXPECT_LT(neko::Abs(actual - expected), tolerance)
      << ToFloat(actual) << " vs " << ToFloat(expected);
}

void ExpectNear(Vec2f actual, Vec2f expected, Scalar tolerance) {
  ExpectNear(actual.x(), expected.x(), tolerance);
  ExpectNear(actual.y(), expected.y(), tolerance);
}

const Scalar kTolerance = Ratio(1, 500);

Scalar Degrees(int degrees) {
  return Radian{neko::Degree{static_cast<Scalar>(degrees)}}.value();
}

// Static ground (top at y = 0, 20 m wide) and helpers to place solid shapes
struct Fixture {
  Fixture() {
    const auto body = AddBody({Scalar{0}, Scalar{-1}});
    world.body(body).type = neko::BodyType::kStatic;
    ground = AddAabb(body, {Scalar{10}, Scalar{1}});
  }
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
  neko::Contact Detect(ColliderIndex collider1, ColliderIndex collider2) {
    auto& c1 = world.collider(collider1);
    auto& c2 = world.collider(collider2);
    neko::Contact contact{};
    EXPECT_TRUE(world.DetectContact(world.body(c1.body_index), c1,
                                    world.body(c2.body_index), c2, &contact));
    return contact;
  }

  neko::PhysicsWorld world{{.gravity = Vec2f::Zero()}};
  ColliderIndex ground{};
};

}  // namespace

TEST(Manifold, FlatBoxOnGroundTouchesAtBothBottomCorners) {
  // A 2 x 1 oriented box sunk 0.1 into the ground: the reference face is the
  // ground's top, the incident face the box's bottom
  Fixture f;
  const auto box = f.AddOrientedBox(f.AddBody({Scalar{3}, Ratio(4, 10)}),
                                    {Scalar{1}, Ratio(1, 2)});
  const auto contact = f.Detect(f.ground, box);
  ASSERT_EQ(contact.point_count, 2);
  ExpectNear(contact.points[0].position, {Scalar{2}, Ratio(-1, 10)},
             kTolerance);
  ExpectNear(contact.points[1].position, {Scalar{4}, Ratio(-1, 10)},
             kTolerance);
  ExpectNear(contact.points[0].penetration, Ratio(1, 10), kTolerance);
  ExpectNear(contact.points[1].penetration, Ratio(1, 10), kTolerance);
}

TEST(Manifold, PointsAreClippedToTheReferenceFace) {
  // A box hanging past the ground's right end (x = 10): its right corner is
  // clipped to the end of the ground
  Fixture f;
  const auto box = f.AddOrientedBox(f.AddBody({Scalar{10}, Ratio(4, 10)}),
                                    {Scalar{1}, Ratio(1, 2)});
  const auto contact = f.Detect(f.ground, box);
  ASSERT_EQ(contact.point_count, 2);
  ExpectNear(contact.points[0].position.x(), Scalar{9}, kTolerance);
  ExpectNear(contact.points[1].position.x(), Scalar{10}, kTolerance);
}

TEST(Manifold, TiltedBoxGetsTwoDepths) {
  // Turned 5 degrees counter-clockwise, center 0.4 above the ground: the
  // bottom corners are 0.5 cos 5 +/- sin 5 below the center, so the left one
  // is 0.1853 deep and the right one 0.0109 deep: two points, two depths
  Fixture f;
  const Scalar angle = Degrees(5);
  const auto body = f.AddBody({Scalar{0}, Ratio(4, 10)}, angle);
  const auto box = f.AddOrientedBox(body, {Scalar{1}, Ratio(1, 2)});
  const auto contact = f.Detect(f.ground, box);
  ASSERT_EQ(contact.point_count, 2);
  // Corner = center + R (-1, -0.5) and R (1, -0.5)
  const auto& rotation = f.world.body(body).rotation;
  const Vec2f left =
      f.world.body(body).position + rotation.Rotate({Scalar{-1}, Ratio(-1, 2)});
  const Vec2f right =
      f.world.body(body).position + rotation.Rotate({Scalar{1}, Ratio(-1, 2)});
  ExpectNear(contact.points[0].position, left, kTolerance);
  ExpectNear(contact.points[1].position, right, kTolerance);
  ExpectNear(contact.points[0].penetration, -left.y(), kTolerance);
  ExpectNear(contact.points[1].penetration, -right.y(), kTolerance);
  EXPECT_GT(contact.points[0].penetration, contact.points[1].penetration);
  // The contact's penetration (SAT's overlap) is the deepest point's depth,
  // computed another way: equal up to rounding
  ExpectNear(contact.penetration, contact.points[0].penetration, kTolerance);
}

TEST(Manifold, CornerLandingTouchesAtOnePoint) {
  // Tilted 20 degrees, only the left corner is below the ground
  Fixture f;
  const auto body = f.AddBody({Scalar{0}, Ratio(75, 100)}, Degrees(20));
  const auto box = f.AddOrientedBox(body, {Scalar{1}, Ratio(1, 2)});
  const auto contact = f.Detect(f.ground, box);
  ASSERT_EQ(contact.point_count, 1);
  ExpectNear(contact.points[0].position, contact.contact_point, kTolerance);
  ExpectNear(contact.points[0].penetration, contact.penetration, kTolerance);
}

TEST(Manifold, DiamondTouchesAtItsCorner) {
  Fixture f;
  const auto box =
      f.AddOrientedBox(f.AddBody({Scalar{0}, Ratio(13, 10)}, Degrees(45)),
                       {Scalar{1}, Scalar{1}});
  const auto contact = f.Detect(f.ground, box);
  ASSERT_EQ(contact.point_count, 1);
  ExpectNear(contact.points[0].position.x(), Scalar{0}, kTolerance);
}

TEST(Manifold, TurnedBoxesFaceToFace) {
  // Two boxes turned 30 degrees together, one on top of the other along
  // their common y axis, overlapping by 0.2: two points
  Fixture f;
  const Scalar angle = Degrees(30);
  const auto lower_body = f.AddBody({Scalar{20}, Scalar{20}}, angle);
  const auto lower = f.AddOrientedBox(lower_body, {Scalar{1}, Ratio(1, 2)});
  const Vec2f up = f.world.body(lower_body).rotation.y_axis();
  const auto upper = f.AddOrientedBox(
      f.AddBody(f.world.body(lower_body).position + up * Ratio(8, 10), angle),
      {Scalar{1}, Ratio(1, 2)});
  const auto contact = f.Detect(upper, lower);
  ASSERT_EQ(contact.point_count, 2);
  ExpectNear(contact.contact_normal, up, kTolerance);
  ExpectNear(contact.points[0].penetration, Ratio(2, 10), kTolerance);
  ExpectNear(contact.points[1].penetration, Ratio(2, 10), kTolerance);
  // The two points are the ends of the shared edge, 2 m apart
  ExpectNear((contact.points[1].position - contact.points[0].position).Length(),
             Scalar{2}, kTolerance);
}

TEST(Manifold, StackedAabbsTouchAlongTheirOverlap) {
  Fixture f;
  const auto box =
      f.AddAabb(f.AddBody({Scalar{9}, Ratio(4, 10)}), {Scalar{2}, Ratio(1, 2)});
  const auto contact = f.Detect(box, f.ground);
  ASSERT_EQ(contact.point_count, 2);
  // On the ground's face (body 2), from x = 7 to the ground's end at x = 10
  EXPECT_EQ(contact.points[0].position, (Vec2f{Scalar{7}, Scalar{0}}));
  EXPECT_EQ(contact.points[1].position, (Vec2f{Scalar{10}, Scalar{0}}));
  EXPECT_EQ(contact.points[0].penetration, contact.penetration);
}

TEST(Manifold, CirclesTouchAtOnePoint) {
  Fixture f;
  const auto body = f.AddBody({Scalar{0}, Ratio(4, 10)});
  const auto circle = f.world.AddCircleCollider(body);
  f.world.collider(circle).is_trigger = false;
  f.world.circle(f.world.collider(circle).shape_index).radius = Ratio(1, 2);
  const auto contact = f.Detect(circle, f.ground);
  ASSERT_EQ(contact.point_count, 1);
  EXPECT_EQ(contact.points[0].position, contact.contact_point);
  EXPECT_EQ(contact.points[0].penetration, contact.penetration);
}

TEST(Manifold, TouchingFlatGivesTwoPointsAtZeroDepth) {
  Fixture f;
  const auto box = f.AddOrientedBox(f.AddBody({Scalar{0}, Ratio(1, 2)}),
                                    {Scalar{1}, Ratio(1, 2)});
  const auto contact = f.Detect(f.ground, box);
  ASSERT_EQ(contact.point_count, 2);
  EXPECT_EQ(contact.points[0].penetration, Scalar{0});
  EXPECT_EQ(contact.points[1].penetration, Scalar{0});
}

TEST(Manifold, TriggerPairsGetNoPoints) {
  Fixture f;
  const auto box = f.AddOrientedBox(f.AddBody({Scalar{0}, Ratio(4, 10)}),
                                    {Scalar{1}, Ratio(1, 2)});
  f.world.collider(box).is_trigger = true;
  const auto contact = f.Detect(f.ground, box);
  EXPECT_EQ(contact.point_count, 0);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
