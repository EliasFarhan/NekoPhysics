#include "tutorials/11_jetpack/solutions/hover_sample.h"

#include "neko/math/func.h"

namespace neko::tutorial {
namespace {
constexpr Scalar kGravity = Ratio(981, 100);
constexpr Scalar kGroundTop = Ratio(1, 2);
constexpr Scalar kPlayerRadius = Ratio(3, 10);
constexpr Scalar kStartX{2};
// Settled: within 1 cm of the target
constexpr Scalar kTolerance = Ratio(1, 100);
// Meters of flame per newton
constexpr Scalar kFlamePerNewton = Ratio(1, 50);
constexpr Scalar kMillimeters{1000};
}  // namespace

void HoverSample::Begin(const SampleContext& context) {
  const Vec2f bounds_center = context.world_bounds.GetCenter();
  const Scalar half_width = context.world_bounds.GetHalfSize().x();
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
  player_ = world_.AddBody();
  Body& player = world_.body(player_);
  player.inverse_mass = Scalar{1} / mass();
  player.inertia_mode = InertiaMode::kFixedRotation;
  player.position = {context.world_bounds.min_bound().x() + kStartX,
                     kGroundTop + kPlayerRadius};
  const ColliderIndex collider_index = world_.AddCircleCollider(player_);
  Collider& collider = world_.collider(collider_index);
  collider.is_trigger = false;
  collider.restitution = Scalar{0};
  world_.circle(collider.shape_index).radius = kPlayerRadius;
  start_height_ = player.position.y();
  step_ = 0;
  last_outside_ = 0;
  overshoot_ = Scalar{0};
}

void HoverSample::FixedUpdate() {
  const Body& player = world_.body(player_);
  const Scalar omega = parameters_[kOmega].ToScalar();
  const Scalar zeta = parameters_[kDampingRatio].ToScalar();
  // The controller: gravity cancelled, then a spring and a damper on the
  // height error, all as an acceleration times the mass
  const Scalar acceleration = kGravity +
                              omega * omega * (target() - player.position.y()) -
                              Scalar{2} * zeta * omega * player.velocity.y();
  thrust_ = Clamp(mass() * acceleration, Scalar{0},
                  parameters_[kMaxThrust].ToScalar());
  world_.ApplyForce(player_, {Scalar{0}, thrust_});
  world_.Step();
  ++step_;

  const Scalar error = height() - target();
  if (step_ <= kMeasuredSteps) {
    overshoot_ = Max(overshoot_, error);
    if (Abs(error) > kTolerance) {
      last_outside_ = step_;
    }
  }
  readouts_[0].value = ToDecimal(thrust_, readouts_[0].scale);
  readouts_[1].value = ToDecimal(height(), readouts_[1].scale);
  readouts_[2].value = ToDecimal(overshoot_ * kMillimeters, readouts_[2].scale);
  readouts_[3].value = ToDecimal(settling_time(), readouts_[3].scale);
  for (auto& readout : readouts_) {
    readout.is_valid = true;
  }
}

Scalar HoverSample::settling_time() const {
  return static_cast<Scalar>(last_outside_) * world_.dt();
}

void HoverSample::Draw(DrawList& draw_list) const {
  draw_list.boxes.push_back(
      {ground_.GetCenter(), ground_.GetHalfSize(), sample_color::kGrey});
  draw_list.lines.push_back({Vec2f{ground_.min_bound().x(), target()},
                             Vec2f{ground_.max_bound().x(), target()},
                             sample_color::kGreen});
  const Body& player = world_.body(player_);
  if (thrust_ > Scalar{0}) {
    const Vec2f nozzle = player.position - Vec2f{Scalar{0}, kPlayerRadius};
    draw_list.lines.push_back(
        {nozzle, nozzle - Vec2f{Scalar{0}, thrust_ * kFlamePerNewton},
         sample_color::kRed});
  }
  draw_list.circles.push_back(
      {player.position, kPlayerRadius, sample_color::kOrange});
}

void HoverSample::End() {
  world_.Clear();
  step_ = 0;
  last_outside_ = 0;
  thrust_ = Scalar{0};
  overshoot_ = Scalar{0};
  for (auto& readout : readouts_) {
    readout.is_valid = false;
  }
}

}  // namespace neko::tutorial
