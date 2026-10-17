#include "tutorials/07_pendulum/solutions/physical_pendulum_sample.h"

#include "neko/math/angle.h"
#include "neko/math/const.h"
#include "neko/math/func.h"

#include "main/samples/body_draw.h"

namespace neko::tutorial {
namespace {
constexpr Scalar kGravity = Ratio(981, 100);
constexpr Scalar kPivotRadius = Ratio(1, 20);
constexpr Scalar kCenterRadius = Ratio(1, 25);
}  // namespace

void PhysicalPendulumSample::Begin(const SampleContext& context) {
  const Vec2f top{context.world_bounds.GetCenter().x(),
                  context.world_bounds.max_bound().y() - Scalar{1}};
  pivot_ = world_.AddBody();
  world_.body(pivot_).type = BodyType::kStatic;
  world_.body(pivot_).inverse_mass = Scalar{0};
  world_.body(pivot_).position = top;

  // The center at theta0 from the vertical, d below the pivot. The local
  // corner (-a/2, a/2) points 45 degrees past the up axis: turning the box
  // by theta0 - 45 degrees puts it on the pivot
  const Scalar half = size() / Scalar{2};
  const Scalar distance = half * Sqrt(Scalar{2});
  const Radian angle = amplitude();
  box_ = world_.AddBody();
  Body& box = world_.body(box_);
  box.inverse_mass = Scalar{1};
  box.position = top + Vec2f{Sin(angle), -Cos(angle)} * distance;
  box.rotation.set_angle(angle - Radian{Degree{Scalar{45}}});
  // A square of 1 kg: I = m a^2 / 6 about its center, set by hand (no
  // collider)
  box.inertia_mode = InertiaMode::kOverride;
  box.inverse_inertia = Scalar{6} / (size() * size());

  pin_ = world_.AddDistanceJoint(pivot_, box_);
  world_.joint(pin_).local_anchor_b = {-half, half};
  world_.joint(pin_).length = Scalar{0};
  previous_x_ = box.position.x() - top.x();
  max_pin_error_ = Scalar{0};

  readouts_[0].value = ToDecimal(predicted_period(), readouts_[0].scale);
  readouts_[2].value = ToDecimal(equivalent_length(), readouts_[2].scale);
  readouts_[0].is_valid = true;
  readouts_[2].is_valid = true;
}

void PhysicalPendulumSample::FixedUpdate() {
  world_.Step();
  ++step_;
  const Scalar x =
      world_.body(box_).position.x() - world_.body(pivot_).position.x();
  if ((previous_x_ < Scalar{0}) != (x < Scalar{0})) {
    const Scalar fraction = previous_x_ / (previous_x_ - x);
    const Scalar time =
        (static_cast<Scalar>(step_ - 1) + fraction) * world_.dt();
    if (crossings_ == 0) {
      first_crossing_ = time;
    }
    last_crossing_ = time;
    ++crossings_;
  }
  previous_x_ = x;
  max_pin_error_ = Max(max_pin_error_, world_.JointDistance(pin_));
  if (const auto period = measured_period()) {
    readouts_[1].value = ToDecimal(*period, readouts_[1].scale);
    readouts_[1].is_valid = true;
  }
  readouts_[3].value =
      ToDecimal(max_pin_error_ * Scalar{1000}, readouts_[3].scale);
  readouts_[3].is_valid = true;
}

void PhysicalPendulumSample::Draw(DrawList& draw_list) const {
  const Body& pivot = world_.body(pivot_);
  const Body& box = world_.body(box_);
  const Scalar half = size() / Scalar{2};
  DrawBoxOutline(box.position, {half, half}, box.rotation,
                 sample_color::kOrange, draw_list);
  draw_list.lines.push_back(
      {pivot.position, box.position, sample_color::kWhite});
  draw_list.circles.push_back(
      {box.position, kCenterRadius, sample_color::kWhite});
  draw_list.circles.push_back(
      {pivot.position, kPivotRadius, sample_color::kGrey});
}

void PhysicalPendulumSample::End() {
  world_.Clear();
  step_ = 0;
  crossings_ = 0;
  first_crossing_ = Scalar{0};
  last_crossing_ = Scalar{0};
  for (auto& readout : readouts_) {
    readout.is_valid = false;
  }
}

Radian PhysicalPendulumSample::amplitude() const {
  return Radian{Degree{static_cast<Scalar>(parameters_[kAmplitude].value)}};
}

Scalar PhysicalPendulumSample::equivalent_length() const {
  // I_p / (m d) = (a^2 / 6 + a^2 / 2) / (a / sqrt 2) = 4 d / 3
  const Scalar distance = size() / Scalar{2} * Sqrt(Scalar{2});
  return Scalar{4} * distance / Scalar{3};
}

Scalar PhysicalPendulumSample::predicted_period() const {
  const Scalar theta = amplitude().value();
  const Scalar theta_square = theta * theta;
  const Scalar series = Scalar{1} + theta_square / Scalar{16} +
                        Scalar{11} * theta_square * theta_square / Scalar{3072};
  return Scalar{2} * Pi<Scalar>() * Sqrt(equivalent_length() / kGravity) *
         series;
}

std::optional<Scalar> PhysicalPendulumSample::measured_period() const {
  if (crossings_ < 3) {
    return std::nullopt;
  }
  return Scalar{2} * (last_crossing_ - first_crossing_) /
         static_cast<Scalar>(crossings_ - 1);
}

}  // namespace neko::tutorial
