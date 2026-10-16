#include "tutorials/05_friction/friction_sample.h"

#include "neko/math/angle.h"

#include "main/samples/body_draw.h"

namespace neko::tutorial {
namespace {
constexpr Vec2f kRampHalfSize{Scalar{7}, Ratio(1, 4)};
constexpr Vec2f kCrateHalfSize{Ratio(1, 2), Ratio(1, 2)};
constexpr Scalar kGravity = Ratio(981, 100);
}  // namespace

void FrictionSample::Begin(const SampleContext& context) {
  const Vec2f center = context.world_bounds.GetCenter();
  const Scalar friction = parameters_[kFriction].ToScalar();
  const Radian angle{
      Degree{static_cast<Scalar>(parameters_[kRampAngle].value)}};

  ramp_index_ = world_.AddBody();
  {
    Body& ramp = world_.body(ramp_index_);
    ramp.type = BodyType::kStatic;
    ramp.inverse_mass = Scalar{0};
    ramp.position = center - Vec2f{Scalar{0}, Scalar{1}};
    ramp.rotation.set_angle(angle);
    const ColliderIndex collider_index =
        world_.AddOrientedBoxCollider(ramp_index_);
    Collider& collider = world_.collider(collider_index);
    collider.is_trigger = false;
    collider.restitution = Scalar{0};
    collider.friction = friction;
    world_.oriented_box(collider.shape_index).half_size = kRampHalfSize;
  }
  const Rotation& rotation = world_.body(ramp_index_).rotation;
  down_slope_ = -rotation.x_axis();

  crate_index_ = world_.AddBody();
  {
    Body& crate = world_.body(crate_index_);
    crate.inverse_mass = Scalar{1};
    crate.rotation.set_angle(angle);
    // Resting on the ramp's top face, near its upper end
    crate.position =
        world_.body(ramp_index_).position +
        rotation.y_axis() * (kRampHalfSize.y() + kCrateHalfSize.y()) -
        down_slope_ * Scalar{5};
    start_ = crate.position;
    const ColliderIndex collider_index =
        world_.AddOrientedBoxCollider(crate_index_);
    Collider& collider = world_.collider(collider_index);
    collider.is_trigger = false;
    collider.restitution = Scalar{0};
    collider.friction = friction;
    world_.oriented_box(collider.shape_index).half_size = kCrateHalfSize;
  }

  readouts_[0].value = ToDecimal(Tan(angle), readouts_[0].scale);
  readouts_[1].value = predicted_holds() ? 1 : 0;
  readouts_[2].value = ToDecimal(predicted_acceleration(), readouts_[2].scale);
  for (std::size_t i = 0; i < 3; ++i) {
    readouts_[i].is_valid = true;
  }
}

void FrictionSample::FixedUpdate() {
  world_.Step();
  ++step_;
  if (step_ == kMeasureSteps) {
    const Scalar time = static_cast<Scalar>(kMeasureSteps) * world_.dt();
    measured_acceleration_ = speed_down_slope() / time;
    readouts_[3].value = ToDecimal(*measured_acceleration_, readouts_[3].scale);
    readouts_[3].is_valid = true;
  }
  readouts_[4].value = ToDecimal(distance(), readouts_[4].scale);
  readouts_[4].is_valid = true;
}

void FrictionSample::Draw(DrawList& draw_list) const {
  const Body& ramp = world_.body(ramp_index_);
  const Body& crate = world_.body(crate_index_);
  DrawBoxOutline(ramp.position, kRampHalfSize, ramp.rotation,
                 sample_color::kGrey, draw_list);
  DrawBoxOutline(crate.position, kCrateHalfSize, crate.rotation,
                 sample_color::kOrange, draw_list);
  DrawOrientation(crate, kCrateHalfSize.x(), sample_color::kWhite, draw_list);
  DrawVelocity(crate, sample_color::kSky, draw_list);
}

void FrictionSample::End() {
  world_.Clear();
  step_ = 0;
  measured_acceleration_.reset();
  readouts_[3].is_valid = false;
  readouts_[4].is_valid = false;
}

bool FrictionSample::predicted_holds() const {
  const Radian angle{
      Degree{static_cast<Scalar>(parameters_[kRampAngle].value)}};
  return Sin(angle) <= parameters_[kFriction].ToScalar() * Cos(angle);
}

Scalar FrictionSample::predicted_acceleration() const {
  if (predicted_holds()) {
    return Scalar{0};
  }
  const Radian angle{
      Degree{static_cast<Scalar>(parameters_[kRampAngle].value)}};
  return kGravity *
         (Sin(angle) - parameters_[kFriction].ToScalar() * Cos(angle));
}

Scalar FrictionSample::distance() const {
  return Vec2f::Dot(world_.body(crate_index_).position - start_, down_slope_);
}

Scalar FrictionSample::speed_down_slope() const {
  return Vec2f::Dot(world_.body(crate_index_).velocity, down_slope_);
}

}  // namespace neko::tutorial
