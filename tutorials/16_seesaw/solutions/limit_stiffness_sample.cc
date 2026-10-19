#include "tutorials/16_seesaw/solutions/limit_stiffness_sample.h"

#include "neko/math/angle.h"
#include "neko/math/const.h"
#include "neko/math/func.h"

#include "main/samples/body_draw.h"
#include "tutorials/16_seesaw/seesaw_sample.h"

namespace neko::tutorial {
namespace {
constexpr Vec2f kPlankHalfSize{Scalar{1}, Ratio(1, 10)};
constexpr Scalar kPlankMass{1};
constexpr Scalar kAxleRadius = Ratio(1, 10);

Radian Limit() { return Radian{Degree{Scalar{30}}}; }

// The plank's inertia about its center, m (w^2 + h^2) / 12
Scalar PlankInertia() {
  const Scalar width = Scalar{2} * kPlankHalfSize.x();
  const Scalar height = Scalar{2} * kPlankHalfSize.y();
  return kPlankMass * (width * width + height * height) / Scalar{12};
}

int Millidegrees(Scalar radians, int scale) {
  return ToDecimal(Degree{Radian{radians}}.value(), scale);
}
}  // namespace

void LimitStiffnessSample::Begin(const SampleContext& context) {
  world_.emplace(
      PhysicsWorldConfig{.gravity = Vec2f::Zero(),
                         .dt = kFixedDeltaTime,
                         .max_bodies = 2,
                         .max_colliders = 1,
                         .max_touching_pairs = 1,
                         .max_broadphase_pairs = 1,
                         .max_joints = 0,
                         .max_revolute_joints = 1,
                         .joint_hertz = parameters_[kHertz].ToScalar()});
  const Vec2f center = context.world_bounds.GetCenter();
  axle_ = AddAxle(*world_, center);
  plank_ = AddPlank(*world_, center, kPlankHalfSize, Scalar{1} / kPlankMass,
                    Scalar{0});
  hinge_ = world_->AddRevoluteJoint(axle_, plank_, center);
  SetSymmetricLimit(*world_, hinge_, Limit());
  readouts_[0].value = Millidegrees(predicted_sink(), readouts_[0].scale);
  readouts_[0].is_valid = true;
}

void LimitStiffnessSample::FixedUpdate() {
  world_->body(plank_).torque += parameters_[kTorque].ToScalar();
  world_->Step();
  const Scalar past =
      world_->RevoluteJointAngle(hinge_).value() - Limit().value();
  sinks_[static_cast<std::size_t>(step_ % kAverageSteps)] = past;
  ++step_;
  max_overshoot_ = Max(max_overshoot_, past);
  readouts_[1].value = Millidegrees(measured_sink(), readouts_[1].scale);
  readouts_[2].value = Millidegrees(max_overshoot_, readouts_[2].scale);
  readouts_[3].value = ToDecimal(limit_torque(), readouts_[3].scale);
  for (std::size_t i = 1; i < readouts_.size(); ++i) {
    readouts_[i].is_valid = true;
  }
}

void LimitStiffnessSample::Draw(DrawList& draw_list) const {
  const Body& axle = world_->body(axle_);
  const Body& plank = world_->body(plank_);
  for (const Radian angle : {Limit(), -Limit()}) {
    const Vec2f reach = Vec2f{Cos(angle), Sin(angle)} * kPlankHalfSize.x();
    draw_list.lines.push_back({axle.position - reach, axle.position + reach,
                               sample_color::kDarkGrey});
  }
  DrawBoxOutline(plank.position, kPlankHalfSize, plank.rotation,
                 sample_color::kWhite, draw_list);
  draw_list.circles.push_back(
      {axle.position, kAxleRadius, sample_color::kGrey});
}

void LimitStiffnessSample::End() {
  world_.reset();
  sinks_ = {};
  step_ = 0;
  max_overshoot_ = Scalar{0};
  for (auto& readout : readouts_) {
    readout.is_valid = false;
  }
}

Scalar LimitStiffnessSample::predicted_sink() const {
  const Scalar omega =
      Scalar{2} * Pi<Scalar>() * parameters_[kHertz].ToScalar();
  // omega^2 alone passes Fixed16's 32767 above 28 Hz: divide by omega I first
  return parameters_[kTorque].ToScalar() / (omega * PlankInertia()) / omega;
}

Scalar LimitStiffnessSample::measured_sink() const {
  const int count = Min(step_, kAverageSteps);
  if (count == 0) {
    return Scalar{0};
  }
  Scalar sum{0};
  for (int i = 0; i < count; ++i) {
    sum += sinks_[static_cast<std::size_t>(i)];
  }
  return sum / static_cast<Scalar>(count);
}

Scalar LimitStiffnessSample::limit_torque() const {
  const RevoluteJoint& joint = world_->revolute_joint(hinge_);
  return (joint.upper_impulse - joint.lower_impulse) / world_->dt();
}

}  // namespace neko::tutorial
