#include "tutorials/13_one_way/one_way_sample.h"

#include <algorithm>

#include "neko/math/angle.h"
#include "neko/math/func.h"

#include "main/samples/body_draw.h"
#include "main/samples/sample_rng.h"

namespace neko::tutorial {
namespace {
constexpr Scalar kGroundTop = Ratio(1, 2);
// The stack of floors over the player, its middle from the left of the world
constexpr Scalar kStackX = Ratio(16, 5);
constexpr Scalar kFloorSpacing = Ratio(8, 5);
constexpr Vec2f kFloorHalfSize{Scalar{1}, Ratio(7, 40)};
// The player: 0.38 m wide, 0.6 m tall
constexpr Vec2f kPlayerHalfSize{Ratio(19, 100), Ratio(3, 10)};
// The shelf and the ghost platform on the right, by their tops
constexpr Scalar kShelfX = Ratio(13, 2);
constexpr Scalar kShelfTop = Ratio(24, 5);
constexpr Vec2f kShelfHalfSize{Ratio(3, 2), Ratio(7, 40)};
constexpr Scalar kGhostX{10};
constexpr Scalar kGhostTop = Ratio(29, 10);
constexpr Vec2f kGhostHalfSize{Ratio(6, 5), Ratio(7, 40)};
constexpr Vec2f kCrateHalfSize{Ratio(1, 5), Ratio(1, 5)};
constexpr Scalar kNormalLength = Ratio(3, 10);
// How close to a floor's resting height counts as standing on it
constexpr Scalar kStandTolerance = Ratio(1, 10);
constexpr std::uint32_t kCrateSeed = 13;
}  // namespace

ColliderIndex AddOneWayPlatform(PhysicsWorld& world, Vec2f center,
                                Vec2f half_size, Radian arc) {
  const BodyIndex body = world.AddBody();
  world.body(body).type = BodyType::kStatic;
  world.body(body).inverse_mass = Scalar{0};
  world.body(body).position = center;
  const ColliderIndex index = world.AddAabbCollider(body);
  Collider& collider = world.collider(index);
  collider.is_trigger = false;
  collider.is_one_way = true;
  collider.one_way_arc = arc;
  collider.restitution = Scalar{0};
  world.aabb(collider.shape_index).half_size = half_size;
  return index;
}

ColliderIndex AddCrate(PhysicsWorld& world, Vec2f center, Vec2f half_size,
                       std::uint32_t user_id) {
  const BodyIndex body = world.AddBody();
  world.body(body).position = center;
  world.body(body).inertia_mode = InertiaMode::kFixedRotation;
  const ColliderIndex index = world.AddAabbCollider(body);
  Collider& collider = world.collider(index);
  collider.is_trigger = false;
  collider.restitution = Scalar{0};
  collider.user_id = user_id;
  world.aabb(collider.shape_index).half_size = half_size;
  return index;
}

void DrawOneWayNormal(const PhysicsWorld& world, ColliderIndex collider,
                      DrawList& draw_list) {
  const Collider& one_way = world.collider(collider);
  const Body& body = world.body(one_way.body_index);
  const Vec2f center = world.ColliderCenter(collider);
  const Scalar half_height =
      one_way.type == ShapeType::kAabb
          ? world.aabb(one_way.shape_index).half_size.y()
          : world.oriented_box(one_way.shape_index).half_size.y();
  const Vec2f normal = body.rotation.Rotate(one_way.one_way_normal);
  const Vec2f top = center + body.rotation.y_axis() * half_height;
  draw_list.lines.push_back(
      {top, top + normal * kNormalLength, sample_color::kYellow});
}

void DrawPassingPairs(const PhysicsWorld& world, DrawList& draw_list) {
  for (const ColliderPair& pair : world.passing_pairs()) {
    draw_list.lines.push_back({world.ColliderCenter(pair.c1()),
                               world.ColliderCenter(pair.c2()),
                               sample_color::kMagenta});
  }
}

void OneWaySample::LandingListener::OnCollisionEnter(const ColliderPair& pair) {
  const Collider& collider1 = world->collider(pair.c1());
  const Collider& collider2 = world->collider(pair.c2());
  // The ids tell the game which entities touched
  const bool has_player =
      collider1.user_id == kPlayerId || collider2.user_id == kPlayerId;
  if (has_player && (collider1.is_one_way || collider2.is_one_way)) {
    ++landings;
  }
}

void OneWaySample::Begin(const SampleContext& context) {
  const Scalar left = context.world_bounds.min_bound().x();
  const Vec2f bounds_center = context.world_bounds.GetCenter();
  const Scalar half_width = context.world_bounds.GetHalfSize().x();
  const Radian arc{Degree{parameters_[kArc].ToScalar()}};
  ground_ = Aabbf::FromCenter({bounds_center.x(), kGroundTop / Scalar{2}},
                              {half_width, kGroundTop / Scalar{2}});
  {
    const BodyIndex ground = world_.AddBody();
    world_.body(ground).type = BodyType::kStatic;
    world_.body(ground).inverse_mass = Scalar{0};
    world_.body(ground).position = ground_.GetCenter();
    const ColliderIndex collider_index = world_.AddAabbCollider(ground);
    Collider& collider = world_.collider(collider_index);
    collider.is_trigger = false;
    collider.restitution = Scalar{0};
    world_.aabb(collider.shape_index).half_size = ground_.GetHalfSize();
  }
  for (std::size_t i = 0; i < floors_.size(); ++i) {
    const Scalar top =
        kGroundTop +
        kFloorSpacing * static_cast<Scalar>(static_cast<int>(i) + 1);
    floors_[i] =
        AddOneWayPlatform(world_, {left + kStackX, top - kFloorHalfSize.y()},
                          kFloorHalfSize, arc);
  }
  shelf_ = AddOneWayPlatform(world_,
                             {left + kShelfX, kShelfTop - kShelfHalfSize.y()},
                             kShelfHalfSize, arc);
  ghost_ = AddOneWayPlatform(world_,
                             {left + kGhostX, kGhostTop - kGhostHalfSize.y()},
                             kGhostHalfSize, arc);
  // Standing on the ground under the stack
  player_ = AddCrate(world_, {left + kStackX, kGroundTop + kPlayerHalfSize.y()},
                     kPlayerHalfSize, kPlayerId);
  // The crates in a row above the shelves, each a little off by the seeded
  // generator, ids 1 to 6
  SampleRng rng{kCrateSeed};
  for (std::size_t i = 0; i < crates_.size(); ++i) {
    const Scalar x = left + Ratio(53, 10) +
                     Ratio(9, 10) * static_cast<Scalar>(static_cast<int>(i)) +
                     rng.Range(Scalar{0}, Ratio(3, 10));
    const Scalar y = Ratio(29, 5) + rng.Range(Scalar{0}, Ratio(4, 5));
    crates_[i] = AddCrate(world_, {x, y}, kCrateHalfSize,
                          static_cast<std::uint32_t>(i + 1));
  }
  listener_.world = &world_;
  listener_.landings = 0;
  world_.SetContactListener(&listener_);
  UpdateReadouts();
}

void OneWaySample::FixedUpdate() {
  // The script: one action per period, jump three times then drop three
  // times
  if (step_ % kActionSteps == kActionOffset) {
    const int action = (step_ / kActionSteps) % 6;
    const BodyIndex player = world_.collider(player_).body_index;
    if (action < 3) {
      Body& body = world_.body(player);
      body.velocity.set_y(parameters_[kJumpSpeed].ToScalar());
    } else if (world_.DropThrough(player) > 0) {
      ++drops_;
    }
  }
  ++step_;
  // The ghost platform blinks on its tick timer
  if (step_ % parameters_[kGhostPeriod].value == 0) {
    Collider& ghost = world_.collider(ghost_);
    ghost.is_enabled = !ghost.is_enabled;
  }
  world_.Step();
  max_passing_ = std::max(max_passing_, world_.passing_pairs().size());
  UpdateReadouts();
}

void OneWaySample::UpdateReadouts() {
  readouts_[0].value = static_cast<int>(world_.passing_pairs().size());
  readouts_[1].value = static_cast<int>(max_passing_);
  const int floor = player_floor();
  readouts_[2].value = floor;
  readouts_[2].is_valid = floor >= 0;
  readouts_[3].value = listener_.landings;
  readouts_[4].value = drops_;
  readouts_[5].value = ghost_enabled() ? 1 : 0;
  for (const std::size_t i : {0u, 1u, 3u, 4u, 5u}) {
    readouts_[i].is_valid = true;
  }
}

int OneWaySample::player_floor() const {
  const Scalar y = player_position().y();
  for (int floor = 0; floor <= 3; ++floor) {
    const Scalar standing = kGroundTop + kPlayerHalfSize.y() +
                            kFloorSpacing * static_cast<Scalar>(floor);
    if (Abs(y - standing) < kStandTolerance) {
      return floor;
    }
  }
  return -1;
}

void OneWaySample::Draw(DrawList& draw_list) const {
  draw_list.boxes.push_back(
      {ground_.GetCenter(), ground_.GetHalfSize(), sample_color::kGrey});
  const auto draw_platform = [this, &draw_list](ColliderIndex index) {
    const Collider& collider = world_.collider(index);
    draw_list.boxes.push_back(
        {world_.ColliderCenter(index),
         world_.aabb(collider.shape_index).half_size,
         collider.is_enabled ? sample_color::kTeal : sample_color::kDarkGrey});
    if (collider.is_enabled) {
      DrawOneWayNormal(world_, index, draw_list);
    }
  };
  for (const ColliderIndex floor : floors_) {
    draw_platform(floor);
  }
  draw_platform(shelf_);
  draw_platform(ghost_);
  const auto draw_box = [this, &draw_list](ColliderIndex index, ColorId color) {
    const Collider& collider = world_.collider(index);
    const Vec2f center = world_.ColliderCenter(index);
    const Vec2f half_size = world_.aabb(collider.shape_index).half_size;
    draw_list.boxes.push_back({center, half_size, color});
    // A shape in a passing pair gets a magenta outline
    for (const ColliderPair& pair : world_.passing_pairs()) {
      if (pair.Contains(index)) {
        DrawBoxOutline(center, half_size, Rotation{}, sample_color::kMagenta,
                       draw_list);
        break;
      }
    }
  };
  for (const ColliderIndex crate : crates_) {
    draw_box(crate, sample_color::kBlue);
  }
  draw_box(player_, sample_color::kOrange);
  DrawPassingPairs(world_, draw_list);
}

void OneWaySample::End() {
  world_.SetContactListener(nullptr);
  world_.Clear();
  step_ = 0;
  drops_ = 0;
  max_passing_ = 0;
  listener_.landings = 0;
  for (auto& readout : readouts_) {
    readout.is_valid = false;
  }
}

}  // namespace neko::tutorial
