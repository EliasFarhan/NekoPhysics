#include "tutorials/05_friction/solutions/mixed_surfaces_sample.h"

#include "neko/math/angle.h"
#include "neko/math/func.h"

#include "main/samples/body_draw.h"

namespace neko::tutorial {
namespace {
constexpr Vec2f kRampHalfSize{Scalar{7}, Ratio(1, 4)};
constexpr Vec2f kCrateHalfSize{Ratio(1, 2), Ratio(1, 2)};
constexpr Scalar kGravity = Ratio(981, 100);

Radian DegreesToRadians(int degrees) {
  return Radian{Degree{static_cast<Scalar>(degrees)}};
}
}  // namespace

void MixedSurfacesSample::Begin(const SampleContext& context) {
  const Vec2f center = context.world_bounds.GetCenter();
  const Radian angle = DegreesToRadians(parameters_[kRampAngle].value);

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
    collider.friction = parameters_[kRampFriction].ToScalar();
    world_.oriented_box(collider.shape_index).half_size = kRampHalfSize;
  }
  const Rotation& rotation = world_.body(ramp_index_).rotation;
  down_slope_ = -rotation.x_axis();

  crate_index_ = world_.AddBody();
  {
    Body& crate = world_.body(crate_index_);
    crate.inverse_mass = Scalar{1};
    crate.rotation.set_angle(angle);
    // Resting on the ramp's top face, near its upper end, as in chapter 05
    crate.position =
        world_.body(ramp_index_).position +
        rotation.y_axis() * (kRampHalfSize.y() + kCrateHalfSize.y()) -
        down_slope_ * Scalar{5};
    const ColliderIndex collider_index =
        world_.AddOrientedBoxCollider(crate_index_);
    Collider& collider = world_.collider(collider_index);
    collider.is_trigger = false;
    collider.restitution = Scalar{0};
    collider.friction = parameters_[kCrateFriction].ToScalar();
    world_.oriented_box(collider.shape_index).half_size = kCrateHalfSize;
  }

  readouts_[0].value = ToDecimal(contact_friction(), readouts_[0].scale);
  readouts_[1].value = steepest_holding_angle();
  readouts_[2].value = predicted_holds() ? 1 : 0;
  readouts_[3].value = ToDecimal(predicted_acceleration(), readouts_[3].scale);
  for (std::size_t i = 0; i < 4; ++i) {
    readouts_[i].is_valid = true;
  }
}

void MixedSurfacesSample::FixedUpdate() {
  world_.Step();
  ++step_;
  if (step_ == kMeasureSteps) {
    const Scalar time = static_cast<Scalar>(kMeasureSteps) * world_.dt();
    measured_acceleration_ =
        Vec2f::Dot(world_.body(crate_index_).velocity, down_slope_) / time;
    readouts_[4].value = ToDecimal(*measured_acceleration_, readouts_[4].scale);
    readouts_[4].is_valid = true;
  }
}

Scalar MixedSurfacesSample::contact_friction() const {
  // The same expression as PhysicsWorld::DetectContact, so the same bits
  return Sqrt(parameters_[kCrateFriction].ToScalar() *
              parameters_[kRampFriction].ToScalar());
}

int MixedSurfacesSample::steepest_holding_angle() const {
  const Scalar mu = contact_friction();
  int steepest = 0;
  for (int degrees = 1; degrees < 90; ++degrees) {
    const Radian angle = DegreesToRadians(degrees);
    if (Sin(angle) > mu * Cos(angle)) {
      break;
    }
    steepest = degrees;
  }
  return steepest;
}

bool MixedSurfacesSample::predicted_holds() const {
  const Radian angle = DegreesToRadians(parameters_[kRampAngle].value);
  return Sin(angle) <= contact_friction() * Cos(angle);
}

Scalar MixedSurfacesSample::predicted_acceleration() const {
  if (predicted_holds()) {
    return Scalar{0};
  }
  const Radian angle = DegreesToRadians(parameters_[kRampAngle].value);
  return kGravity * (Sin(angle) - contact_friction() * Cos(angle));
}

void MixedSurfacesSample::Draw(DrawList& draw_list) const {
  const Body& ramp = world_.body(ramp_index_);
  const Body& crate = world_.body(crate_index_);
  DrawBoxOutline(ramp.position, kRampHalfSize, ramp.rotation,
                 sample_color::kGrey, draw_list);
  DrawBoxOutline(crate.position, kCrateHalfSize, crate.rotation,
                 sample_color::kOrange, draw_list);
  DrawOrientation(crate, kCrateHalfSize.x(), sample_color::kWhite, draw_list);
  DrawVelocity(crate, sample_color::kSky, draw_list);
}

void MixedSurfacesSample::End() {
  world_.Clear();
  step_ = 0;
  measured_acceleration_.reset();
  readouts_[4].is_valid = false;
}

}  // namespace neko::tutorial
