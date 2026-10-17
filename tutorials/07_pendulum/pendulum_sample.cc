#include "tutorials/07_pendulum/pendulum_sample.h"

#include "neko/math/angle.h"
#include "neko/math/const.h"
#include "neko/math/func.h"

#include "main/samples/body_draw.h"

namespace neko::tutorial {
namespace {
constexpr Scalar kGravity = Ratio(981, 100);
// PhysicsWorldConfig::joint_hertz and joint_damping_ratio, the defaults
constexpr Scalar kJointHertz{15};
constexpr Scalar kJointDampingRatio{2};
constexpr Scalar kBobRadius = Ratio(3, 20);
constexpr Scalar kPivotRadius = Ratio(1, 20);
// A rope shorter than its length by more than this is slack
constexpr Scalar kSlackTolerance = Ratio(1, 100);
}  // namespace

void PendulumSample::Begin(const SampleContext& context) {
  const Vec2f top{context.world_bounds.GetCenter().x(),
                  context.world_bounds.max_bound().y() - Scalar{1}};
  pivot_ = world_.AddBody();
  world_.body(pivot_).type = BodyType::kStatic;
  world_.body(pivot_).inverse_mass = Scalar{0};
  world_.body(pivot_).position = top;

  // A point mass: no collider, so no inertia, and the joint is anchored at
  // its center, so it never spins
  bob_ = world_.AddBody();
  world_.body(bob_).inverse_mass = Scalar{1};
  const Radian angle = amplitude();
  world_.body(bob_).position = top + Vec2f{Sin(angle), -Cos(angle)} * length();

  joint_ = world_.AddDistanceJoint(pivot_, bob_);
  world_.joint(joint_).length = length();
  world_.joint(joint_).type = parameters_[kRope].value != 0
                                  ? DistanceJointType::kRope
                                  : DistanceJointType::kRigid;
  previous_x_ = world_.body(bob_).position.x() - top.x();

  readouts_[0].value = ToDecimal(predicted_period(), readouts_[0].scale);
  readouts_[0].is_valid = true;
  readouts_[4].value =
      ToDecimal(predicted_max_stretch() * Scalar{1000}, readouts_[4].scale);
  readouts_[4].is_valid = true;
}

void PendulumSample::FixedUpdate() {
  world_.Step();
  ++step_;
  const Scalar dt = world_.dt();
  const Scalar x =
      world_.body(bob_).position.x() - world_.body(pivot_).position.x();
  // Crossing the vertical: interpolate the time between the two steps
  if ((previous_x_ < Scalar{0}) != (x < Scalar{0})) {
    const Scalar fraction = previous_x_ / (previous_x_ - x);
    const Scalar time = (static_cast<Scalar>(step_ - 1) + fraction) * dt;
    if (crossings_ == 0) {
      first_crossing_ = time;
    }
    last_crossing_ = time;
    ++crossings_;
  }
  previous_x_ = x;

  const Scalar error = world_.JointDistance(joint_) - length();
  max_length_error_ = Max(max_length_error_, Abs(error));
  if (error < -kSlackTolerance) {
    ++slack_steps_;
  }

  if (const auto period = measured_period()) {
    readouts_[1].value = ToDecimal(*period, readouts_[1].scale);
    readouts_[1].is_valid = true;
  }
  readouts_[2].value = ToDecimal(Abs(error) * Scalar{1000}, readouts_[2].scale);
  readouts_[3].value =
      ToDecimal(max_length_error_ * Scalar{1000}, readouts_[3].scale);
  readouts_[5].value = slack_steps_;
  readouts_[2].is_valid = true;
  readouts_[3].is_valid = true;
  readouts_[5].is_valid = true;
}

void PendulumSample::Draw(DrawList& draw_list) const {
  const Body& pivot = world_.body(pivot_);
  const Body& bob = world_.body(bob_);
  const bool is_rope = parameters_[kRope].value != 0;
  const bool is_slack =
      world_.JointDistance(joint_) < length() - kSlackTolerance;
  const ColorId link_color = !is_rope   ? sample_color::kWhite
                             : is_slack ? sample_color::kDarkGrey
                                        : sample_color::kYellow;
  draw_list.lines.push_back({pivot.position, bob.position, link_color});
  draw_list.circles.push_back(
      {pivot.position, kPivotRadius, sample_color::kGrey});
  draw_list.circles.push_back(
      {bob.position, kBobRadius, sample_color::kOrange});
  DrawVelocity(bob, sample_color::kSky, draw_list);
}

void PendulumSample::End() {
  world_.Clear();
  step_ = 0;
  crossings_ = 0;
  first_crossing_ = Scalar{0};
  last_crossing_ = Scalar{0};
  max_length_error_ = Scalar{0};
  slack_steps_ = 0;
  for (auto& readout : readouts_) {
    readout.is_valid = false;
  }
}

Radian PendulumSample::amplitude() const {
  return Radian{Degree{static_cast<Scalar>(parameters_[kAmplitude].value)}};
}

Scalar PendulumSample::predicted_period() const {
  const Scalar theta = amplitude().value();
  const Scalar theta_square = theta * theta;
  const Scalar series = Scalar{1} + theta_square / Scalar{16} +
                        Scalar{11} * theta_square * theta_square / Scalar{3072};
  return Scalar{2} * Pi<Scalar>() * Sqrt(length() / kGravity) * series;
}

std::optional<Scalar> PendulumSample::measured_period() const {
  if (crossings_ < 3) {
    return std::nullopt;
  }
  // Two crossings per period
  return Scalar{2} * (last_crossing_ - first_crossing_) /
         static_cast<Scalar>(crossings_ - 1);
}

Scalar PendulumSample::predicted_max_stretch() const {
  const Scalar omega = Scalar{2} * Pi<Scalar>() * kJointHertz;
  const Scalar cos_amplitude = Cos(amplitude());
  // The tension at the bottom over the soft step stiffness
  const Scalar sag =
      kGravity * (Scalar{3} - Scalar{2} * cos_amplitude) / (omega * omega);
  // Each step moves the bob v dt along the tangent, (v dt)^2 / (2 L) off the
  // circle, and the soft step pulls back h w / (2 zeta + h w) of the error
  // per step: the error settles where both balance
  const Scalar dt = world_.dt();
  const Scalar speed_square =
      Scalar{2} * kGravity * length() * (Scalar{1} - cos_amplitude);
  const Scalar drift = speed_square * dt * dt / (Scalar{2} * length());
  const Scalar correction =
      dt * omega / (Scalar{2} * kJointDampingRatio + dt * omega);
  return sag + drift / correction;
}

}  // namespace neko::tutorial
