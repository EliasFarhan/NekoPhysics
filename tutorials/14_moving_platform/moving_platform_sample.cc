#include "tutorials/14_moving_platform/moving_platform_sample.h"

#include "neko/math/angle.h"
#include "neko/math/const.h"
#include "neko/math/func.h"

#include "main/samples/body_draw.h"
#include "tutorials/13_one_way/one_way_sample.h"

namespace neko::tutorial {
namespace {
constexpr int kStepsPerSecond = 60;
constexpr Scalar kGroundTop = Ratio(1, 2);
constexpr Vec2f kPlatformHalfSize{Ratio(4, 5), Ratio(3, 20)};
constexpr Vec2f kCrateHalfSize{Ratio(1, 5), Ratio(1, 5)};
// The player of chapter 13: 0.38 m wide, 0.6 m tall
constexpr Vec2f kPlayerHalfSize{Ratio(19, 100), Ratio(3, 10)};
// The two patrols of the bottom row and the rider's platform, by their
// centers from the left of the world
constexpr Vec2f kLiftCenter{Ratio(13, 5), Ratio(8, 5)};
constexpr Vec2f kLagCenter{Ratio(51, 5), Ratio(8, 5)};
constexpr Scalar kPatrolAmplitude = Ratio(6, 5);
constexpr Vec2f kRiderCenter{Ratio(32, 5), Ratio(17, 5)};
constexpr Scalar kRiderAmplitude = Ratio(8, 5);
// The TicTac platform: its speed, half size and the band of its heights
constexpr Scalar kTicTacSpeed = Ratio(8, 5);
constexpr Vec2f kTicTacHalfSize{Ratio(3, 5), Ratio(3, 20)};
constexpr Scalar kTicTacLow{5};
constexpr Scalar kTicTacHigh = Ratio(32, 5);
constexpr Scalar kTicTacStartX{2};

// The crate (or player) resting on the top of a platform centered on
// `center` with half height `half_height`
Vec2f OnTop(Vec2f center, Scalar half_height, Vec2f rider_half_size) {
  return {center.x(), center.y() + half_height + rider_half_size.y()};
}

// The millimeters a readout shows for `meters`, with `scale` steps per mm
int Millimeters(Scalar meters, int scale) {
  return ToDecimal(meters * Scalar{1000}, scale);
}
}  // namespace

ColliderIndex AddKinematicPlatform(PhysicsWorld& world, Vec2f center,
                                   Vec2f half_size) {
  const BodyIndex body = world.AddBody();
  world.body(body).type = BodyType::kKinematic;
  world.body(body).inverse_mass = Scalar{0};
  world.body(body).position = center;
  const ColliderIndex index = world.AddAabbCollider(body);
  Collider& collider = world.collider(index);
  collider.is_trigger = false;
  collider.restitution = Scalar{0};
  world.aabb(collider.shape_index).half_size = half_size;
  return index;
}

Vec2f PatrolPoint(Vec2f center, Scalar amplitude, int step, int period_steps) {
  const Radian phase{Scalar{2} * Pi<Scalar>() *
                     static_cast<Scalar>(step % period_steps) /
                     static_cast<Scalar>(period_steps)};
  return center - Vec2f{amplitude * Cos(phase), Scalar{0}};
}

bool IsTouching(const PhysicsWorld& world, ColliderPair pair) {
  for (const TouchingPair& touching : world.touching_pairs()) {
    if (touching.pair == pair) {
      return true;
    }
  }
  return false;
}

void DriveLagging(PhysicsWorld& world, BodyIndex platform, Vec2f target,
                  Scalar tau) {
  Body& body = world.body(platform);
  // Unity's PatrolPlatform: velocity = deltaPos, with tau = 1 s. A speed
  // proportional to the distance left: the platform never reaches a
  // moving target
  body.velocity = (target - body.position) / tau;
}

void MovingPlatformSample::ExitListener::OnCollisionExit(
    const ColliderPair& pair) {
  if (world->collider(pair.c1()).user_id == kTicTacCrateId ||
      world->collider(pair.c2()).user_id == kTicTacCrateId) {
    ++exits;
  }
}

Vec2f MovingPlatformSample::LiftPath(int step) const {
  return PatrolPoint(lift_center_, kPatrolAmplitude, step, period_steps_);
}

Vec2f MovingPlatformSample::LagPath(int step) const {
  return PatrolPoint(lag_center_, kPatrolAmplitude, step, period_steps_);
}

void MovingPlatformSample::Begin(const SampleContext& context) {
  left_ = context.world_bounds.min_bound().x();
  right_ = context.world_bounds.max_bound().x();
  const Vec2f bounds_center = context.world_bounds.GetCenter();
  const Scalar half_width = context.world_bounds.GetHalfSize().x();
  period_steps_ = parameters_[kPeriod].value * kStepsPerSecond;
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
  const Vec2f origin{left_, context.world_bounds.min_bound().y()};
  lift_center_ = origin + kLiftCenter;
  lag_center_ = origin + kLagCenter;
  rider_center_ = origin + kRiderCenter;
  // Every patrol starts at rest at the left end of its path
  const Vec2f lift_start = LiftPath(0);
  lift_ = AddKinematicPlatform(world_, lift_start, kPlatformHalfSize);
  lift_crate_ =
      AddCrate(world_, OnTop(lift_start, kPlatformHalfSize.y(), kCrateHalfSize),
               kCrateHalfSize, 1);
  const Vec2f lag_start = LagPath(0);
  lag_ = AddKinematicPlatform(world_, lag_start, kPlatformHalfSize);
  lag_crate_ =
      AddCrate(world_, OnTop(lag_start, kPlatformHalfSize.y(), kCrateHalfSize),
               kCrateHalfSize, 3);
  const Vec2f rider_start =
      PatrolPoint(rider_center_, kRiderAmplitude, 0, period_steps_);
  rider_platform_ =
      AddKinematicPlatform(world_, rider_start, kPlatformHalfSize);
  // Frictionless: only the game's carry moves it along
  player_ = AddCrate(world_,
                     OnTop(rider_start, kPlatformHalfSize.y(), kPlayerHalfSize),
                     kPlayerHalfSize, kPlayerId);
  world_.collider(player_).friction = Scalar{0};
  const Vec2f tictac_center{left_ + kTicTacStartX,
                            rng_.Range(kTicTacLow, kTicTacHigh)};
  tictac_ = AddKinematicPlatform(world_, tictac_center, kTicTacHalfSize);
  tictac_crate_ = AddCrate(
      world_, OnTop(tictac_center, kTicTacHalfSize.y(), kCrateHalfSize),
      kCrateHalfSize, kTicTacCrateId);
  crate_start_offset_ = position(lift_crate_).x() - position(lift_).x();
  listener_.world = &world_;
  listener_.exits = 0;
  world_.SetContactListener(&listener_);
  UpdateReadouts();
}

void MovingPlatformSample::WrapTicTac() {
  const BodyIndex platform = world_.collider(tictac_).body_index;
  const Vec2f from = world_.body(platform).position;
  if (from.x() - kTicTacHalfSize.x() <= right_) {
    return;
  }
  // Back to the left edge, at a new seeded height
  const Vec2f to{left_ - kTicTacHalfSize.x(),
                 rng_.Range(kTicTacLow, kTicTacHigh)};
  const Vec2f offset = to - from;
  // The riders first, while the touching pairs still describe the old place:
  // the same offset keeps them where they stand on the platform
  const BodyIndex crate = world_.collider(tictac_crate_).body_index;
  if (IsTouching(world_, {tictac_, tictac_crate_})) {
    const Body& body = world_.body(crate);
    world_.Teleport(crate, body.position + offset, body.rotation.angle());
  }
  world_.Teleport(platform, to, Radian{});
  ++wraps_;
}

void MovingPlatformSample::FixedUpdate() {
  // A wrap drops the pending target: wrap first, then set the target
  WrapTicTac();
  const BodyIndex tictac = world_.collider(tictac_).body_index;
  world_.SetKinematicTarget(tictac,
                            world_.body(tictac).position +
                                Vec2f{kTicTacSpeed * world_.dt(), Scalar{0}},
                            Radian{});
  world_.SetKinematicTarget(world_.collider(lift_).body_index,
                            LiftPath(step_ + 1), Radian{});
  DriveLagging(world_, world_.collider(lag_).body_index, LagPath(step_ + 1),
               parameters_[kLagTau].ToScalar());
  const BodyIndex rider_platform = world_.collider(rider_platform_).body_index;
  world_.SetKinematicTarget(
      rider_platform,
      PatrolPoint(rider_center_, kRiderAmplitude, step_ + 1, period_steps_),
      Radian{});
  // The game-side carry: the frictionless player gets the platform's
  // horizontal velocity while it stands on it (touching since the last step)
  if (parameters_[kRiderCarry].value != 0 &&
      IsTouching(world_, {rider_platform_, player_})) {
    Body& player = world_.body(world_.collider(player_).body_index);
    player.velocity.set_x(world_.body(rider_platform).velocity.x());
  }
  world_.Step();
  ++step_;
  UpdateReadouts();
}

Scalar MovingPlatformSample::crate_slip() const {
  return position(lift_crate_).x() - position(lift_).x() - crate_start_offset_;
}

Scalar MovingPlatformSample::rider_offset() const {
  return position(player_).x() - position(rider_platform_).x();
}

void MovingPlatformSample::UpdateReadouts() {
  readouts_[0].value = Millimeters((position(lift_) - LiftPath(step_)).Length(),
                                   readouts_[0].scale);
  readouts_[1].value = Millimeters((position(lag_) - LagPath(step_)).Length(),
                                   readouts_[1].scale);
  readouts_[2].value = Millimeters(crate_slip(), readouts_[2].scale);
  readouts_[3].value = Millimeters(rider_offset(), readouts_[3].scale);
  readouts_[4].value = wraps_;
  readouts_[5].value = listener_.exits;
  for (auto& readout : readouts_) {
    readout.is_valid = true;
  }
}

void MovingPlatformSample::Draw(DrawList& draw_list) const {
  draw_list.boxes.push_back(
      {ground_.GetCenter(), ground_.GetHalfSize(), sample_color::kGrey});
  draw_list.boxes.push_back(
      {position(lift_), kPlatformHalfSize, sample_color::kTeal});
  draw_list.boxes.push_back(
      {position(lag_), kPlatformHalfSize, sample_color::kPurple});
  // Where the lagging platform should be
  DrawBoxOutline(LagPath(step_), kPlatformHalfSize, Rotation{},
                 sample_color::kYellow, draw_list);
  draw_list.boxes.push_back(
      {position(rider_platform_), kPlatformHalfSize, sample_color::kTeal});
  draw_list.boxes.push_back(
      {position(tictac_), kTicTacHalfSize, sample_color::kLime});
  for (const ColliderIndex crate : {lift_crate_, lag_crate_, tictac_crate_}) {
    draw_list.boxes.push_back(
        {position(crate), kCrateHalfSize, sample_color::kBlue});
  }
  draw_list.boxes.push_back(
      {position(player_), kPlayerHalfSize, sample_color::kOrange});
}

void MovingPlatformSample::End() {
  world_.SetContactListener(nullptr);
  world_.Clear();
  rng_ = SampleRng{14};
  step_ = 0;
  wraps_ = 0;
  listener_.exits = 0;
  for (auto& readout : readouts_) {
    readout.is_valid = false;
  }
}

}  // namespace neko::tutorial
