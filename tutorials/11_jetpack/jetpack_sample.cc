#include "tutorials/11_jetpack/jetpack_sample.h"

#include "neko/math/func.h"

#include "main/samples/body_draw.h"

namespace neko::tutorial {
namespace {
constexpr Scalar kGravity = Ratio(981, 100);
constexpr Scalar kGroundTop = Ratio(1, 2);
constexpr Scalar kPlayerRadius = Ratio(3, 10);
constexpr Scalar kStartX{2};
// Height of the wind above the ground top, in meters
constexpr Scalar kWindAltitude = Ratio(3, 2);
// Drag coefficient of the wind, in N s/m
constexpr Scalar kWindDrag{2};
constexpr Scalar kFlameLength = Ratio(1, 2);
constexpr int kWindArrows = 6;
constexpr Scalar kArrowTime = Ratio(1, 5);
}  // namespace

void JetpackSample::Begin(const SampleContext& context) {
  const Vec2f bounds_center = context.world_bounds.GetCenter();
  const Scalar half_width = context.world_bounds.GetHalfSize().x();
  ground_ = Aabbf::FromCenter({bounds_center.x(), kGroundTop / Scalar{2}},
                              {half_width, kGroundTop / Scalar{2}});
  wind_height_ = kGroundTop + kWindAltitude;
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
  player_ = world_.AddBody();
  {
    Body& player = world_.body(player_);
    player.inverse_mass = Scalar{1} / mass();
    player.inertia_mode = InertiaMode::kFixedRotation;
    player.gravity_scale = gravity_scale();
    // Standing on the ground, touching it
    player.position = {context.world_bounds.min_bound().x() + kStartX,
                       kGroundTop + kPlayerRadius};
    const ColliderIndex collider_index = world_.AddCircleCollider(player_);
    Collider& collider = world_.collider(collider_index);
    collider.is_trigger = false;
    collider.restitution = Scalar{0};
    world_.circle(collider.shape_index).radius = kPlayerRadius;
  }
  readouts_[0].value = ToDecimal(hover_thrust(), readouts_[0].scale);
  readouts_[1].value = ToDecimal(predicted_acceleration(), readouts_[1].scale);
  readouts_[5].value = ToDecimal(predicted_kick(), readouts_[5].scale);
  for (const std::size_t i : {0u, 1u, 5u}) {
    readouts_[i].is_valid = true;
  }
}

void JetpackSample::FixedUpdate() {
  ++step_;
  const Body& player = world_.body(player_);
  const Scalar inverse_mass = player.inverse_mass;
  const Scalar dt = world_.dt();

  // The controller: every source adds its force for this step; the
  // knockback's lock is the one priority rule
  thrusting_ =
      step_ >= kThrustStart && step_ < kThrustEnd && step_ > lock_until_;
  if (thrusting_) {
    world_.ApplyForce(player_, {Scalar{0}, parameters_[kThrust].ToScalar()});
  }
  // The drift so far, before this step's wind: wind_steps_ steps of it
  const Scalar velocity_before = player.velocity.x();
  if (step_ == kKnockbackStep) {
    measured_drift_ = velocity_before;
    drift_steps_ = wind_steps_;
    readouts_[3].value = ToDecimal(predicted_drift(), readouts_[3].scale);
    readouts_[4].value = ToDecimal(*measured_drift_, readouts_[4].scale);
    readouts_[3].is_valid = true;
    readouts_[4].is_valid = true;
  }
  Scalar wind_force{};
  if (player.position.y() > wind_height_) {
    wind_force =
        kWindDrag * (parameters_[kWind].ToScalar() - player.velocity.x());
    world_.ApplyForce(player_, {wind_force, Scalar{0}});
    ++wind_steps_;
  }
  if (step_ == kKnockbackStep) {
    world_.ApplyImpulse(player_,
                        {-parameters_[kKnockback].ToScalar(), Scalar{0}});
    lock_until_ = step_ + kKnockbackLock - 1;
  }

  world_.Step();

  const Body& after = world_.body(player_);
  if (step_ == kKnockbackStep) {
    // The step's change minus what the wind force did: the impulse's share,
    // computed like the integration does
    measured_kick_ =
        after.velocity.x() - velocity_before - wind_force * inverse_mass * dt;
    readouts_[6].value = ToDecimal(*measured_kick_, readouts_[6].scale);
    readouts_[6].is_valid = true;
  }
  if (step_ == kAccelerationFrom) {
    velocity_at_from_ = after.velocity.y();
  } else if (step_ == kAccelerationTo) {
    measured_acceleration_ =
        (after.velocity.y() - velocity_at_from_) /
        (static_cast<Scalar>(kAccelerationTo - kAccelerationFrom) * dt);
    readouts_[2].value = ToDecimal(*measured_acceleration_, readouts_[2].scale);
    readouts_[2].is_valid = true;
  }
}

void JetpackSample::Draw(DrawList& draw_list) const {
  draw_list.boxes.push_back(
      {ground_.GetCenter(), ground_.GetHalfSize(), sample_color::kGrey});
  // The wind line, with arrows as long as the wind travels in 0.2 s
  const Scalar left = ground_.min_bound().x();
  const Scalar right = ground_.max_bound().x();
  draw_list.lines.push_back({Vec2f{left, wind_height_},
                             Vec2f{right, wind_height_}, sample_color::kSky});
  const Scalar wind = parameters_[kWind].ToScalar();
  for (int i = 1; i <= kWindArrows; ++i) {
    const Vec2f from{left + (right - left) * static_cast<Scalar>(i) /
                                static_cast<Scalar>(kWindArrows + 1),
                     wind_height_ + Ratio(1, 2)};
    draw_list.lines.push_back(
        {from, from + Vec2f{wind * kArrowTime, Scalar{0}}, sample_color::kSky});
  }
  const Body& player = world_.body(player_);
  if (thrusting_) {
    const Vec2f nozzle = player.position - Vec2f{Scalar{0}, kPlayerRadius};
    draw_list.lines.push_back(
        {nozzle, nozzle - Vec2f{Scalar{0}, kFlameLength}, sample_color::kRed});
  }
  draw_list.circles.push_back(
      {player.position, kPlayerRadius, sample_color::kOrange});
  DrawVelocity(player, sample_color::kWhite, draw_list);
}

void JetpackSample::End() {
  world_.Clear();
  step_ = 0;
  wind_steps_ = 0;
  drift_steps_ = 0;
  lock_until_ = -1;
  thrusting_ = false;
  velocity_at_from_ = Scalar{0};
  measured_acceleration_.reset();
  measured_drift_.reset();
  measured_kick_.reset();
  for (auto& readout : readouts_) {
    readout.is_valid = false;
  }
}

Scalar JetpackSample::hover_thrust() const {
  return mass() * kGravity * gravity_scale();
}

Scalar JetpackSample::predicted_acceleration() const {
  return parameters_[kThrust].ToScalar() / mass() - kGravity * gravity_scale();
}

Scalar JetpackSample::predicted_drift() const {
  // The gap to the wind speed shrinks by the same factor each step
  const Scalar shrink = Scalar{1} - kWindDrag * kFixedDeltaTime / mass();
  Scalar gap{1};
  for (int i = 0; i < drift_steps_; ++i) {
    gap = gap * shrink;
  }
  return parameters_[kWind].ToScalar() * (Scalar{1} - gap);
}

Scalar JetpackSample::predicted_kick() const {
  return -parameters_[kKnockback].ToScalar() / mass();
}

}  // namespace neko::tutorial
