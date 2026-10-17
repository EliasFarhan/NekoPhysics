// Collision filtering and world queries (M1): category and mask, overlap
// queries and raycasts against circles, boxes and oriented boxes, checked
// against hand-computed hits.
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <type_traits>

#include <gtest/gtest.h>

#include "neko/math/angle.h"
#include "neko/physics/physics.h"

namespace {

using neko::Aabbf;
using neko::BodyIndex;
using neko::ColliderIndex;
using neko::QueryFilter;
using neko::RaycastHit;
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

constexpr float kTolerance = 1e-3f;

Vec2f V(int x, int y) {
  return {static_cast<Scalar>(x), static_cast<Scalar>(y)};
}

// A world without gravity, so nothing moves unless pushed
neko::PhysicsWorld MakeWorld() {
  return neko::PhysicsWorld{{.gravity = Vec2f::Zero()}};
}

BodyIndex AddBody(neko::PhysicsWorld& world, Vec2f position) {
  const BodyIndex body = world.AddBody();
  world.body(body).position = position;
  return body;
}

ColliderIndex AddCircle(neko::PhysicsWorld& world, BodyIndex body,
                        Scalar radius) {
  const ColliderIndex collider = world.AddCircleCollider(body);
  world.collider(collider).is_trigger = false;
  world.circle(world.collider(collider).shape_index).radius = radius;
  return collider;
}

ColliderIndex AddBox(neko::PhysicsWorld& world, BodyIndex body,
                     Vec2f half_size) {
  const ColliderIndex collider = world.AddAabbCollider(body);
  world.collider(collider).is_trigger = false;
  world.aabb(world.collider(collider).shape_index).half_size = half_size;
  return collider;
}

ColliderIndex AddOrientedBox(neko::PhysicsWorld& world, BodyIndex body,
                             Vec2f half_size, float degrees) {
  const ColliderIndex collider = world.AddOrientedBoxCollider(body);
  world.collider(collider).is_trigger = false;
  world.oriented_box(world.collider(collider).shape_index).half_size =
      half_size;
  world.body(body).rotation.set_angle(
      neko::Radian{Scalar{degrees * 3.14159265f / 180.0f}});
  return collider;
}

void ExpectNear(Vec2f actual, float x, float y) {
  EXPECT_NEAR(ToFloat(actual.x()), x, kTolerance);
  EXPECT_NEAR(ToFloat(actual.y()), y, kTolerance);
}

// --- Filtering ---

TEST(Filter, ShouldCollideIsSymmetric) {
  neko::Collider a;
  neko::Collider b;
  EXPECT_TRUE(neko::ShouldCollide(a, b));  // defaults meet
  a.category = 0b0010;
  b.mask = 0b1101;  // b refuses a's category
  EXPECT_FALSE(neko::ShouldCollide(a, b));
  EXPECT_FALSE(neko::ShouldCollide(b, a));
  b.mask = 0xFFFF;
  a.mask = 0b0010;  // a refuses b's category 1
  EXPECT_FALSE(neko::ShouldCollide(a, b));
  b.category = 0b0110;  // now shares a bit with a's mask
  EXPECT_TRUE(neko::ShouldCollide(a, b));
}

class CountingListener final : public neko::ContactListener {
 public:
  void OnTriggerEnter(const neko::ColliderPair&) override { ++triggers; }
  void OnTriggerExit(const neko::ColliderPair&) override {}
  void OnCollisionEnter(const neko::ColliderPair&) override { ++collisions; }
  void OnCollisionExit(const neko::ColliderPair&) override {}
  int triggers = 0;
  int collisions = 0;
};

// Two overlapping solid circles: pushed apart with the default filter,
// untouched and unreported when one mask leaves the other's category out
TEST(Filter, FilteredPairDoesNothing) {
  for (const bool filtered : {false, true}) {
    auto world = MakeWorld();
    CountingListener listener;
    world.SetContactListener(&listener);
    const BodyIndex left = AddBody(world, V(0, 0));
    const BodyIndex right = AddBody(world, {Ratio(3, 2), Scalar{0}});
    const ColliderIndex a = AddCircle(world, left, Scalar{1});
    AddCircle(world, right, Scalar{1});
    if (filtered) {
      world.collider(a).mask = 0xFFFE;  // not category 1
    }
    world.Step();
    EXPECT_EQ(listener.collisions, filtered ? 0 : 1);
    EXPECT_EQ(world.touching_pairs().size(), filtered ? 0u : 1u);
    if (filtered) {
      EXPECT_EQ(world.body(left).velocity, Vec2f::Zero());
      EXPECT_EQ(world.body(right).velocity, Vec2f::Zero());
    } else {
      EXPECT_LT(ToFloat(world.body(left).velocity.x()), 0.0f);
    }
  }
}

TEST(Filter, FilteredTriggerSendsNoEvent) {
  for (const bool filtered : {false, true}) {
    auto world = MakeWorld();
    CountingListener listener;
    world.SetContactListener(&listener);
    const ColliderIndex zone =
        AddCircle(world, AddBody(world, V(0, 0)), Scalar{2});
    world.collider(zone).is_trigger = true;
    const ColliderIndex player =
        AddCircle(world, AddBody(world, V(1, 0)), Scalar{1});
    if (filtered) {
      world.collider(player).category = 0b0100;
      world.collider(zone).mask = 0b0011;
    }
    world.Step();
    EXPECT_EQ(listener.triggers, filtered ? 0 : 1);
  }
}

// The filter is collider state: hashed once it leaves its defaults, and
// copied for rollback
TEST(Filter, ChecksumAndCopy) {
  auto world = MakeWorld();
  const ColliderIndex collider =
      AddCircle(world, AddBody(world, V(0, 0)), Scalar{1});
  const std::uint32_t unfiltered = world.GenerateChecksum();
  world.collider(collider).category = 2;
  const std::uint32_t with_category = world.GenerateChecksum();
  EXPECT_NE(with_category, unfiltered);
  world.collider(collider).mask = 0x00FF;
  EXPECT_NE(world.GenerateChecksum(), with_category);
  auto copy = MakeWorld();
  copy.CopyFrom(world);
  EXPECT_EQ(copy.GenerateChecksum(), world.GenerateChecksum());
  EXPECT_EQ(copy.collider(collider).mask, 0x00FF);
}

// --- Overlap queries ---

TEST(Overlap, EmptyWorld) {
  const auto world = MakeWorld();
  std::array<ColliderIndex, 4> hits{};
  EXPECT_EQ(world.OverlapCircle(V(0, 0), Scalar{10}, hits), 0u);
  EXPECT_EQ(world.OverlapAabb(Aabbf{V(-10, -10), V(10, 10)}, hits), 0u);
}

// One circle query against each shape, at distances where it overlaps,
// touches exactly or misses by a little
TEST(Overlap, CircleAgainstEachShape) {
  auto world = MakeWorld();
  const ColliderIndex circle =
      AddCircle(world, AddBody(world, V(3, 0)), Scalar{1});  // touches at x = 2
  const ColliderIndex box =
      AddBox(world, AddBody(world, V(0, 3)), V(1, 1));  // touches at y = 2
  // A diamond centered at (-4, 0): its corner at x = -4 + sqrt(2) = -2.586
  const ColliderIndex diamond =
      AddOrientedBox(world, AddBody(world, V(-4, 0)), V(1, 1), 45.0f);
  std::array<ColliderIndex, 4> hits{};
  ASSERT_EQ(world.OverlapCircle(V(0, 0), Scalar{2}, hits), 2u);
  EXPECT_EQ(hits[0], circle);
  EXPECT_EQ(hits[1], box);
  // From (-2, 0): 0.6 m reaches past the diamond's corner, 0.58 m doesn't
  ASSERT_EQ(world.OverlapCircle(V(-2, 0), Ratio(6, 10), hits), 1u);
  EXPECT_EQ(hits[0], diamond);
  EXPECT_EQ(world.OverlapCircle(V(-2, 0), Ratio(58, 100), hits), 0u);
}

// The box query is exact against an oriented box: a box in the corner of
// the diamond's bounding box doesn't touch the diamond
TEST(Overlap, AabbAgainstEachShape) {
  auto world = MakeWorld();
  const ColliderIndex diamond =
      AddOrientedBox(world, AddBody(world, V(0, 0)), V(1, 1), 45.0f);
  const ColliderIndex circle =
      AddCircle(world, AddBody(world, V(5, 0)), Scalar{1});
  const ColliderIndex box = AddBox(world, AddBody(world, V(0, 5)), V(1, 1));
  std::array<ColliderIndex, 4> hits{};
  // In the corner of the diamond's bounding box, (1.41, 1.41), but 0.7 m
  // from the diamond's face x + y = sqrt(2)
  const Aabbf corner{{Ratio(12, 10), Ratio(12, 10)}, V(2, 2)};
  EXPECT_EQ(world.OverlapAabb(corner, hits), 0u);
  const Aabbf middle{V(-1, -1), V(1, 1)};
  ASSERT_EQ(world.OverlapAabb(middle, hits), 1u);
  EXPECT_EQ(hits[0], diamond);
  // Over the diamond, touching the circle at (4, 0) and the box at (1, 4)
  const Aabbf wide{V(1, -1), V(4, 4)};
  ASSERT_EQ(world.OverlapAabb(wide, hits), 3u);
  EXPECT_EQ(hits[0], diamond);
  EXPECT_EQ(hits[1], circle);
  EXPECT_EQ(hits[2], box);
}

TEST(Overlap, FilterCategoryTriggersAndInactiveBodies) {
  auto world = MakeWorld();
  const ColliderIndex solid =
      AddCircle(world, AddBody(world, V(0, 0)), Scalar{1});
  const ColliderIndex other_category =
      AddCircle(world, AddBody(world, V(0, 0)), Scalar{1});
  world.collider(other_category).category = 0b0100;
  const ColliderIndex trigger =
      AddCircle(world, AddBody(world, V(0, 0)), Scalar{1});
  world.collider(trigger).is_trigger = true;
  const BodyIndex sleeping = AddBody(world, V(0, 0));
  AddCircle(world, sleeping, Scalar{1});
  world.body(sleeping).is_active = false;

  std::array<ColliderIndex, 4> hits{};
  ASSERT_EQ(world.OverlapCircle(V(0, 0), Scalar{1}, hits), 2u);
  EXPECT_EQ(hits[0], solid);
  EXPECT_EQ(hits[1], other_category);
  ASSERT_EQ(world.OverlapCircle(V(0, 0), Scalar{1}, hits, {.mask = 0b0100}),
            1u);
  EXPECT_EQ(hits[0], other_category);
  ASSERT_EQ(
      world.OverlapCircle(V(0, 0), Scalar{1}, hits, {.include_triggers = true}),
      3u);
  EXPECT_EQ(hits[2], trigger);
}

// Two colliders of one body are two hits: the game decides what that means
TEST(Overlap, DuplicateCollidersOnOneBody) {
  auto world = MakeWorld();
  const BodyIndex body = AddBody(world, V(0, 0));
  const ColliderIndex first = AddCircle(world, body, Scalar{1});
  const ColliderIndex second = AddBox(world, body, V(1, 1));
  std::array<ColliderIndex, 4> hits{};
  ASSERT_EQ(world.OverlapCircle(V(0, 0), Scalar{1}, hits), 2u);
  EXPECT_EQ(hits[0], first);
  EXPECT_EQ(hits[1], second);
  EXPECT_EQ(world.collider(hits[0]).body_index,
            world.collider(hits[1]).body_index);
}

// More overlaps than room: the count says so and the first ones are kept
TEST(Overlap, Overflow) {
  auto world = MakeWorld();
  std::array<ColliderIndex, 5> colliders{};
  for (int i = 0; i < 5; ++i) {
    colliders[static_cast<std::size_t>(i)] =
        AddCircle(world, AddBody(world, V(i, 0)), Scalar{1});
  }
  std::array<ColliderIndex, 2> hits{};
  EXPECT_EQ(world.OverlapCircle(V(2, 0), Scalar{10}, hits), 5u);
  EXPECT_EQ(hits[0], colliders[0]);
  EXPECT_EQ(hits[1], colliders[1]);
  // An empty buffer only counts
  EXPECT_EQ(world.OverlapCircle(V(2, 0), Scalar{10}, {}), 5u);
}

// Queries read the bodies as they are now, not the last broadphase
TEST(Overlap, SeesTeleportBeforeStep) {
  auto world = MakeWorld();
  const BodyIndex body = AddBody(world, V(0, 0));
  AddCircle(world, body, Scalar{1});
  world.Step();
  world.body(body).position = V(20, 0);
  std::array<ColliderIndex, 1> hits{};
  EXPECT_EQ(world.OverlapCircle(V(0, 0), Scalar{1}, hits), 0u);
  EXPECT_EQ(world.OverlapCircle(V(20, 0), Scalar{1}, hits), 1u);
}

// --- Raycasts ---

TEST(Raycast, EmptyWorld) {
  const auto world = MakeWorld();
  std::array<RaycastHit, 2> hits{};
  EXPECT_EQ(world.Raycast(V(0, 0), V(10, 0), hits), 0u);
  EXPECT_FALSE(world.RaycastClosest(V(0, 0), V(10, 0)).has_value());
}

// From (-5, 0) to (5, 0) into a unit circle at the origin: enters at x = -1,
// 4 m along a 10 m segment
TEST(Raycast, Circle) {
  auto world = MakeWorld();
  const ColliderIndex circle =
      AddCircle(world, AddBody(world, V(0, 0)), Scalar{1});
  const auto hit = world.RaycastClosest(V(-5, 0), V(5, 0));
  ASSERT_TRUE(hit.has_value());
  EXPECT_EQ(hit->collider, circle);
  EXPECT_NEAR(ToFloat(hit->fraction), 0.4f, kTolerance);
  ExpectNear(hit->point, -1.0f, 0.0f);
  ExpectNear(hit->normal, -1.0f, 0.0f);
  // An oblique 10 m segment through the center enters 1 m before it
  const auto oblique = world.RaycastClosest(V(-3, -4), V(3, 4));
  ASSERT_TRUE(oblique.has_value());
  EXPECT_NEAR(ToFloat(oblique->fraction), 0.4f, kTolerance);  // 4 of 10 m
  ExpectNear(oblique->point, -0.6f, -0.8f);
  ExpectNear(oblique->normal, -0.6f, -0.8f);
}

// A segment along y = 1 grazes the unit circle at its top: touching counts
TEST(Raycast, TangentCircle) {
  auto world = MakeWorld();
  AddCircle(world, AddBody(world, V(0, 0)), Scalar{1});
  const auto hit = world.RaycastClosest(V(-5, 1), V(5, 1));
  ASSERT_TRUE(hit.has_value());
  EXPECT_NEAR(ToFloat(hit->fraction), 0.5f, kTolerance);
  ExpectNear(hit->point, 0.0f, 1.0f);
  ExpectNear(hit->normal, 0.0f, 1.0f);
  EXPECT_FALSE(world
                   .RaycastClosest({Scalar{-5}, Ratio(101, 100)},
                                   {Scalar{5}, Ratio(101, 100)})
                   .has_value());
}

TEST(Raycast, Box) {
  auto world = MakeWorld();
  const ColliderIndex box = AddBox(world, AddBody(world, V(3, 0)), V(1, 1));
  const auto hit = world.RaycastClosest(V(0, 0), V(10, 0));
  ASSERT_TRUE(hit.has_value());
  EXPECT_EQ(hit->collider, box);
  EXPECT_NEAR(ToFloat(hit->fraction), 0.2f, kTolerance);
  ExpectNear(hit->point, 2.0f, 0.0f);
  ExpectNear(hit->normal, -1.0f, 0.0f);
  // From above, going down: the top face
  const auto down = world.RaycastClosest(V(3, 5), V(3, -5));
  ASSERT_TRUE(down.has_value());
  EXPECT_NEAR(ToFloat(down->fraction), 0.4f, kTolerance);
  ExpectNear(down->normal, 0.0f, 1.0f);
  // Along the bottom side y = -1: grazes it
  const auto grazing = world.RaycastClosest(V(0, -1), V(10, -1));
  ASSERT_TRUE(grazing.has_value());
  EXPECT_NEAR(ToFloat(grazing->fraction), 0.2f, kTolerance);
}

// Through the corner exactly: both sides are entered at the same t, and the
// x side wins the tie
TEST(Raycast, BoxCornerTieTakesX) {
  auto world = MakeWorld();
  AddBox(world, AddBody(world, V(3, 3)), V(1, 1));
  const auto hit = world.RaycastClosest(V(0, 0), V(4, 4));
  ASSERT_TRUE(hit.has_value());
  EXPECT_NEAR(ToFloat(hit->fraction), 0.5f, kTolerance);
  ExpectNear(hit->point, 2.0f, 2.0f);
  ExpectNear(hit->normal, -1.0f, 0.0f);
}

// A unit box turned 30 degrees: the horizontal ray through its center
// enters the face whose normal points at 210 degrees, 1 / cos(30) = 1.1547 m
// left of the center
TEST(Raycast, OrientedBox) {
  auto world = MakeWorld();
  const ColliderIndex box =
      AddOrientedBox(world, AddBody(world, V(0, 0)), V(1, 1), 30.0f);
  const auto hit = world.RaycastClosest(V(-5, 0), V(5, 0));
  ASSERT_TRUE(hit.has_value());
  EXPECT_EQ(hit->collider, box);
  EXPECT_NEAR(ToFloat(hit->fraction), (5.0f - 1.1547f) / 10.0f, kTolerance);
  ExpectNear(hit->point, -1.1547f, 0.0f);
  ExpectNear(hit->normal, -0.8660f, -0.5f);
}

// Starting inside a shape: a hit at t = 0, at the start, without a normal
TEST(Raycast, StartInside) {
  auto world = MakeWorld();
  AddCircle(world, AddBody(world, V(0, 0)), Scalar{1});
  AddBox(world, AddBody(world, V(10, 0)), V(1, 1));
  AddOrientedBox(world, AddBody(world, V(20, 0)), V(1, 1), 30.0f);
  for (const int x : {0, 10, 20}) {
    const Vec2f start{static_cast<Scalar>(x) + Ratio(1, 2), Scalar{0}};
    const auto hit = world.RaycastClosest(start, start + V(0, 3));
    ASSERT_TRUE(hit.has_value()) << x;
    EXPECT_EQ(hit->fraction, Scalar{0});
    EXPECT_EQ(hit->normal, Vec2f::Zero());
    EXPECT_EQ(hit->point, start);
  }
}

// A segment of length zero is a point test
TEST(Raycast, ZeroLength) {
  auto world = MakeWorld();
  AddCircle(world, AddBody(world, V(0, 0)), Scalar{1});
  AddBox(world, AddBody(world, V(10, 0)), V(1, 1));
  EXPECT_TRUE(world.RaycastClosest(V(0, 0), V(0, 0)).has_value());
  EXPECT_TRUE(world.RaycastClosest(V(10, 1), V(10, 1)).has_value());
  EXPECT_FALSE(world.RaycastClosest(V(5, 0), V(5, 0)).has_value());
}

TEST(Raycast, MissesShortAndAway) {
  auto world = MakeWorld();
  AddCircle(world, AddBody(world, V(0, 0)), Scalar{1});
  AddBox(world, AddBody(world, V(10, 0)), V(1, 1));
  // Ends before the shape
  EXPECT_FALSE(world.RaycastClosest(V(-5, 0), V(-2, 0)).has_value());
  EXPECT_FALSE(world.RaycastClosest(V(5, 0), V(8, 0)).has_value());
  // Points away, the shapes behind the start
  EXPECT_FALSE(world.RaycastClosest(V(2, 0), V(5, 0)).has_value());
  EXPECT_FALSE(world.RaycastClosest(V(12, 0), V(15, 0)).has_value());
}

// A direction component of one Fixed16 step doesn't overflow the slab test
TEST(Raycast, NearlyHorizontal) {
  auto world = MakeWorld();
  AddBox(world, AddBody(world, V(3, 0)), V(1, 1));
  const Vec2f end{Scalar{10}, std::numeric_limits<Scalar>::epsilon()};
  const auto hit = world.RaycastClosest(V(0, 0), end);
  ASSERT_TRUE(hit.has_value());
  EXPECT_NEAR(ToFloat(hit->fraction), 0.2f, kTolerance);
  ExpectNear(hit->normal, -1.0f, 0.0f);
}

// Hits sorted by fraction whatever the table order; equal fractions by
// collider index; a short buffer keeps the nearest
TEST(Raycast, SortedTiesAndOverflow) {
  auto world = MakeWorld();
  const ColliderIndex farthest =
      AddCircle(world, AddBody(world, V(8, 0)), Scalar{1});
  const ColliderIndex nearest = AddBox(world, AddBody(world, V(2, 0)), V(1, 1));
  const ColliderIndex middle_a =
      AddCircle(world, AddBody(world, V(5, 0)), Scalar{1});
  const ColliderIndex middle_b =
      AddCircle(world, AddBody(world, V(5, 0)), Scalar{1});
  std::array<RaycastHit, 4> hits{};
  ASSERT_EQ(world.Raycast(V(-10, 0), V(10, 0), hits), 4u);
  EXPECT_EQ(hits[0].collider, nearest);
  EXPECT_EQ(hits[1].collider, middle_a);
  EXPECT_EQ(hits[2].collider, middle_b);
  EXPECT_EQ(hits[3].collider, farthest);
  EXPECT_EQ(hits[1].fraction, hits[2].fraction);
  for (std::size_t i = 1; i < hits.size(); ++i) {
    EXPECT_LE(hits[i - 1].fraction, hits[i].fraction);
  }
  std::array<RaycastHit, 2> two{};
  ASSERT_EQ(world.Raycast(V(-10, 0), V(10, 0), two), 4u);
  EXPECT_EQ(two[0].collider, nearest);
  EXPECT_EQ(two[1].collider, middle_a);
  // From the other side the order turns around, ties still by index
  ASSERT_EQ(world.Raycast(V(10, 0), V(-10, 0), hits), 4u);
  EXPECT_EQ(hits[0].collider, farthest);
  EXPECT_EQ(hits[1].collider, middle_a);
  EXPECT_EQ(hits[2].collider, middle_b);
  EXPECT_EQ(hits[3].collider, nearest);
  EXPECT_EQ(world.Raycast(V(-10, 0), V(10, 0), {}), 4u);
}

// Line of sight: a trigger and a filtered-out category don't block the view
TEST(Raycast, Filter) {
  auto world = MakeWorld();
  const ColliderIndex zone =
      AddCircle(world, AddBody(world, V(2, 0)), Scalar{1});
  world.collider(zone).is_trigger = true;
  const ColliderIndex glass = AddBox(world, AddBody(world, V(5, 0)), V(1, 1));
  world.collider(glass).category = 0b0100;
  const ColliderIndex wall = AddBox(world, AddBody(world, V(8, 0)), V(1, 1));
  world.collider(wall).category = 0b0010;
  auto hit = world.RaycastClosest(V(0, 0), V(10, 0), {.mask = 0b0010});
  ASSERT_TRUE(hit.has_value());
  EXPECT_EQ(hit->collider, wall);
  hit = world.RaycastClosest(V(0, 0), V(10, 0));
  ASSERT_TRUE(hit.has_value());
  EXPECT_EQ(hit->collider, glass);
  hit = world.RaycastClosest(V(0, 0), V(10, 0), {.include_triggers = true});
  ASSERT_TRUE(hit.has_value());
  EXPECT_EQ(hit->collider, zone);
}

// Queries change nothing: same checksum before and after
TEST(Query, DoesNotChangeTheWorld) {
  auto world = MakeWorld();
  AddCircle(world, AddBody(world, V(0, 0)), Scalar{1});
  AddBox(world, AddBody(world, V(3, 0)), V(1, 1));
  world.Step();
  const std::uint32_t before = world.GenerateChecksum();
  std::array<ColliderIndex, 4> overlaps{};
  std::array<RaycastHit, 4> hits{};
  static_cast<void>(world.OverlapCircle(V(0, 0), Scalar{5}, overlaps));
  static_cast<void>(world.Raycast(V(-5, 0), V(5, 0), hits));
  EXPECT_EQ(world.GenerateChecksum(), before);
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
