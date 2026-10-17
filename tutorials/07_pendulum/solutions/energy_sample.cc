#include "tutorials/07_pendulum/solutions/energy_sample.h"

#include "neko/math/angle.h"
#include "neko/math/func.h"

#include "main/samples/body_draw.h"

namespace neko::tutorial {
namespace {
constexpr Scalar kGravity = Ratio(981, 100);
constexpr Scalar kBobRadius = Ratio(3, 20);
constexpr Scalar kPivotRadius = Ratio(1, 20);
}  // namespace

void EnergySample::Begin(const SampleContext& context) {
  world_ = PhysicsWorld{
      {.gravity = Vec2f{Scalar{0}, -kGravity},
       .dt = kFixedDeltaTime,
       .max_bodies = 2,
       .max_colliders = 1,
       .max_touching_pairs = 1,
       .max_broadphase_pairs = 1,
       .max_joints = 1,
       .joint_hertz = static_cast<Scalar>(parameters_[kJointHertz].value)}};
  const Vec2f top{context.world_bounds.GetCenter().x(),
                  context.world_bounds.max_bound().y() - Scalar{1}};
  pivot_ = world_.AddBody();
  world_.body(pivot_).type = BodyType::kStatic;
  world_.body(pivot_).inverse_mass = Scalar{0};
  world_.body(pivot_).position = top;

  bob_ = world_.AddBody();
  world_.body(bob_).inverse_mass = Scalar{1};
  const Radian angle{
      Degree{static_cast<Scalar>(parameters_[kAmplitude].value)}};
  world_.body(bob_).position = top + Vec2f{Sin(angle), -Cos(angle)} * length();

  rod_ = world_.AddDistanceJoint(pivot_, bob_);
  world_.joint(rod_).length = length();
  start_energy_ = energy();
  previous_energy_ = start_energy_;
  previous_x_ = world_.body(bob_).position.x() - top.x();
  crossings_ = 0;
  readouts_[0].value = ToDecimal(start_energy_, readouts_[0].scale);
  readouts_[0].is_valid = true;
}

void EnergySample::FixedUpdate() {
  world_.Step();
  const Scalar x =
      world_.body(bob_).position.x() - world_.body(pivot_).position.x();
  const Scalar now = energy();
  // Crossing the vertical: interpolate the energy between the two steps
  if ((previous_x_ < Scalar{0}) != (x < Scalar{0})) {
    const Scalar fraction = previous_x_ / (previous_x_ - x);
    const Scalar crossing =
        previous_energy_ + (now - previous_energy_) * fraction;
    if (crossings_ == 0) {
      first_bottom_energy_ = crossing;
    }
    last_bottom_energy_ = crossing;
    ++crossings_;
  }
  previous_x_ = x;
  previous_energy_ = now;

  if (crossings_ > 0) {
    readouts_[1].value = ToDecimal(first_bottom_energy_, readouts_[1].scale);
    readouts_[2].value = ToDecimal(last_bottom_energy_, readouts_[2].scale);
    readouts_[1].is_valid = true;
    readouts_[2].is_valid = true;
  }
  if (const auto loss = loss_per_period()) {
    readouts_[3].value = ToDecimal(*loss, readouts_[3].scale);
    readouts_[3].is_valid = true;
  }
  readouts_[4].value =
      ToDecimal(length_error() * Scalar{1000}, readouts_[4].scale);
  readouts_[4].is_valid = true;
}

std::optional<Scalar> EnergySample::loss_per_period() const {
  if (crossings_ < 3) {
    return std::nullopt;
  }
  // Two crossings per period
  const Scalar periods = static_cast<Scalar>(crossings_ - 1) / Scalar{2};
  return (first_bottom_energy_ - last_bottom_energy_) / first_bottom_energy_ *
         Scalar{100} / periods;
}

Scalar EnergySample::energy() const {
  const Body& bob = world_.body(bob_);
  const Scalar bottom = world_.body(pivot_).position.y() - length();
  // A 1 kg bob
  return Vec2f::Dot(bob.velocity, bob.velocity) / Scalar{2} +
         kGravity * (bob.position.y() - bottom);
}

Scalar EnergySample::length_error() const {
  return world_.JointDistance(rod_) - length();
}

void EnergySample::Draw(DrawList& draw_list) const {
  const Body& pivot = world_.body(pivot_);
  const Body& bob = world_.body(bob_);
  draw_list.lines.push_back(
      {pivot.position, bob.position, sample_color::kWhite});
  const Scalar bottom = pivot.position.y() - length();
  draw_list.lines.push_back({Vec2f{pivot.position.x() - Ratio(1, 2), bottom},
                             Vec2f{pivot.position.x() + Ratio(1, 2), bottom},
                             sample_color::kGreen});
  draw_list.circles.push_back(
      {pivot.position, kPivotRadius, sample_color::kGrey});
  draw_list.circles.push_back(
      {bob.position, kBobRadius, sample_color::kOrange});
  DrawVelocity(bob, sample_color::kSky, draw_list);
}

void EnergySample::End() {
  world_.Clear();
  crossings_ = 0;
  for (auto& readout : readouts_) {
    readout.is_valid = false;
  }
}

}  // namespace neko::tutorial
