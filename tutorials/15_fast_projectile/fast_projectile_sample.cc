#include "tutorials/15_fast_projectile/fast_projectile_sample.h"

#include "neko/math/angle.h"
#include "neko/math/const.h"
#include "neko/math/func.h"

namespace neko::tutorial {
namespace {
// Lanes from the bottom of the world: the two water lanes, then the bullet
// and the plain solid lanes
constexpr std::array<Scalar, FastProjectileSample::kLaneCount> kLaneHeights{
    Ratio(29, 5), Ratio(22, 5), Ratio(14, 5), Ratio(7, 5)};
// The walls and players stand 4 m from the left of the world, the shots start
// 0.6 m to 1.1 m from it
constexpr Scalar kTargetX{4};
constexpr Scalar kMuzzleX = Ratio(3, 5);
constexpr Scalar kMuzzleSpread = Ratio(1, 2);
constexpr Scalar kWallHalfHeight = Ratio(1, 2);
// The player of chapter 13 (0.38 m by 0.6 m), walking up and down 0.8 m
// around its lane, one cycle every 97 steps (not a multiple of the firing
// period, so every firing finds it somewhere else)
constexpr Vec2f kPlayerHalfSize{Ratio(19, 100), Ratio(3, 10)};
constexpr Scalar kPlayerAmplitude = Ratio(4, 5);
constexpr int kPlayerPeriod = 97;

// The player's height around its lane after `step` steps
Scalar PlayerOffset(int step) {
  const Radian phase{Scalar{2} * Pi<Scalar>() *
                     static_cast<Scalar>(step % kPlayerPeriod) /
                     static_cast<Scalar>(kPlayerPeriod)};
  return kPlayerAmplitude * Sin(phase);
}
}  // namespace

ColliderIndex AddShot(PhysicsWorld& world, Vec2f center, bool is_bullet,
                      bool is_trigger) {
  const BodyIndex body = world.AddBody();
  Body& shot = world.body(body);
  shot.position = center;
  shot.gravity_scale = Scalar{0};
  shot.inertia_mode = InertiaMode::kFixedRotation;
  shot.is_bullet = is_bullet;
  const ColliderIndex index = world.AddCircleCollider(body);
  Collider& collider = world.collider(index);
  collider.is_trigger = is_trigger;
  collider.restitution = Scalar{0};
  collider.friction = Scalar{0};
  world.circle(collider.shape_index).radius = kShotRadius;
  return index;
}

ColliderIndex AddWall(PhysicsWorld& world, Vec2f center, Vec2f half_size) {
  const BodyIndex body = world.AddBody();
  world.body(body).type = BodyType::kStatic;
  world.body(body).inverse_mass = Scalar{0};
  world.body(body).position = center;
  const ColliderIndex index = world.AddAabbCollider(body);
  Collider& collider = world.collider(index);
  collider.is_trigger = false;
  collider.restitution = Scalar{0};
  world.aabb(collider.shape_index).half_size = half_size;
  return index;
}

void Fire(PhysicsWorld& world, ColliderIndex collider, Vec2f from,
          Vec2f velocity) {
  const BodyIndex body = world.collider(collider).body_index;
  world.Teleport(body, from, Radian{});
  world.body(body).velocity = velocity;
}

void FastProjectileSample::HitListener::OnTriggerEnter(
    const ColliderPair& pair) {
  for (std::size_t player = 0; player < 2; ++player) {
    const std::size_t lane = kWaterBulletLane + player;
    if (pair.Contains(sample->shots_[lane]) &&
        pair.Contains(sample->players_[player])) {
      ++sample->hits_[lane];
      sample->hit_step_[player] = sample->step_;
    }
  }
}

void FastProjectileSample::Begin(const SampleContext& context) {
  const Vec2f origin = context.world_bounds.min_bound();
  muzzle_x_ = origin.x() + kMuzzleX;
  wall_x_ = origin.x() + kTargetX;
  for (std::size_t lane = 0; lane < kLaneCount; ++lane) {
    lane_y_[lane] = origin.y() + kLaneHeights[lane];
  }
  const Scalar half_thickness = thickness() / Scalar{2};
  for (const std::size_t lane : {kPlainLane, kBulletLane}) {
    walls_[lane] = AddWall(world_, {wall_x_, lane_y_[lane]},
                           {half_thickness, kWallHalfHeight});
  }
  for (std::size_t player = 0; player < 2; ++player) {
    const BodyIndex body = world_.AddBody();
    world_.body(body).type = BodyType::kKinematic;
    world_.body(body).inverse_mass = Scalar{0};
    world_.body(body).position = {wall_x_, lane_y_[kWaterBulletLane + player]};
    players_[player] = world_.AddAabbCollider(body);
    Collider& collider = world_.collider(players_[player]);
    collider.is_trigger = false;
    world_.aabb(collider.shape_index).half_size = kPlayerHalfSize;
  }
  for (std::size_t lane = 0; lane < kLaneCount; ++lane) {
    const bool is_bullet = lane == kBulletLane || lane == kWaterBulletLane;
    const bool is_water = lane >= kWaterBulletLane;
    shots_[lane] =
        AddShot(world_, {muzzle_x_, lane_y_[lane]}, is_bullet, is_water);
  }
  hit_step_ = {-kFirePeriod, -kFirePeriod};
  listener_.sample = this;
  world_.SetContactListener(&listener_);
  FireAll();
  UpdateReadouts();
}

void FastProjectileSample::FireAll() {
  // The outcome of the last firing of the solid lanes: stopped before the
  // wall's middle, or past it
  if (firings_ > 0) {
    for (const std::size_t lane : {kPlainLane, kBulletLane}) {
      if (shot_position(lane).x() < wall_x_) {
        ++hits_[lane];
      } else {
        ++misses_[lane];
      }
    }
  }
  const Scalar start = muzzle_x_ + rng_.Range(Scalar{0}, kMuzzleSpread);
  for (std::size_t lane = 0; lane < kLaneCount; ++lane) {
    Fire(world_, shots_[lane], {start, lane_y_[lane]}, {speed(), Scalar{0}});
  }
  ++firings_;
}

void FastProjectileSample::MovePlayers() {
  for (std::size_t player = 0; player < 2; ++player) {
    const BodyIndex body = world_.collider(players_[player]).body_index;
    world_.SetKinematicTarget(
        body,
        {wall_x_, lane_y_[kWaterBulletLane + player] + PlayerOffset(step_ + 1)},
        Radian{});
  }
}

void FastProjectileSample::FixedUpdate() {
  if (step_ > 0 && step_ % kFirePeriod == 0) {
    FireAll();
  }
  for (std::size_t lane = 0; lane < kLaneCount; ++lane) {
    previous_[lane] = shot_position(lane);
  }
  MovePlayers();
  world_.Step();
  ++step_;
  UpdateReadouts();
}

void FastProjectileSample::UpdateReadouts() {
  readouts_[0].value = ToDecimal(speed() * world_.dt(), readouts_[0].scale);
  readouts_[1].value = hits_[kPlainLane];
  readouts_[2].value = misses_[kPlainLane];
  readouts_[3].value = hits_[kBulletLane];
  readouts_[4].value = misses_[kBulletLane];
  readouts_[5].value = hits_[kWaterBulletLane];
  readouts_[6].value = hits_[kWaterPlainLane];
  readouts_[7].value = firings_;
  for (auto& readout : readouts_) {
    readout.is_valid = true;
  }
}

void FastProjectileSample::Draw(DrawList& draw_list) const {
  for (const ColliderIndex wall : walls_) {
    draw_list.boxes.push_back(
        {world_.ColliderCenter(wall),
         world_.aabb(world_.collider(wall).shape_index).half_size,
         sample_color::kGrey});
  }
  for (std::size_t player = 0; player < 2; ++player) {
    const bool was_hit = step_ - hit_step_[player] <= 1;
    draw_list.boxes.push_back(
        {world_.ColliderCenter(players_[player]), kPlayerHalfSize,
         was_hit ? sample_color::kLime : sample_color::kPurple});
  }
  constexpr std::array<ColorId, kLaneCount> shot_colors{
      sample_color::kOrange, sample_color::kSky, sample_color::kCyan,
      sample_color::kCyan};
  for (std::size_t lane = 0; lane < kLaneCount; ++lane) {
    draw_list.circles.push_back(
        {shot_position(lane), kShotRadius, shot_colors[lane]});
    // The step's jump: what the world sees of the way between two steps
    draw_list.lines.push_back(
        {previous_[lane], shot_position(lane), sample_color::kYellow});
  }
}

void FastProjectileSample::End() {
  world_.SetContactListener(nullptr);
  world_.Clear();
  rng_ = SampleRng{15};
  hits_ = {};
  misses_ = {};
  firings_ = 0;
  step_ = 0;
  for (auto& readout : readouts_) {
    readout.is_valid = false;
  }
}

}  // namespace neko::tutorial
