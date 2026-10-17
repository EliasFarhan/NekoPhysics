#include "tutorials/08_chain/solutions/one_step_by_hand_sample.h"

#include "neko/math/const.h"

namespace neko::tutorial {
namespace {
constexpr Scalar kGravity = Ratio(981, 100);
constexpr Scalar kLength{1};
constexpr Scalar kBobRadius = Ratio(3, 20);
constexpr Scalar kPivotRadius = Ratio(1, 20);
}  // namespace

void OneStepByHandSample::Begin(const SampleContext& context) {
  world_ = PhysicsWorld{
      {.gravity = Vec2f{Scalar{0}, -kGravity},
       .dt = kFixedDeltaTime,
       .max_bodies = 2,
       .max_colliders = 1,
       .max_touching_pairs = 1,
       .max_broadphase_pairs = 1,
       .max_joints = 1,
       .velocity_iterations = parameters_[kPasses].value,
       .joint_hertz = static_cast<Scalar>(parameters_[kHertz].value),
       .joint_damping_ratio = parameters_[kDamping].ToScalar()}};
  const Vec2f top{context.world_bounds.GetCenter().x(),
                  context.world_bounds.max_bound().y() - Scalar{2}};
  pivot_ = world_.AddBody();
  world_.body(pivot_).type = BodyType::kStatic;
  world_.body(pivot_).inverse_mass = Scalar{0};
  world_.body(pivot_).position = top;

  const Scalar offset = Ratio(parameters_[kOffset].value, 1000);
  bob_ = world_.AddBody();
  world_.body(bob_).inverse_mass = Scalar{1};
  world_.body(bob_).position = top + Vec2f{Scalar{0}, -(kLength + offset)};
  rod_ = world_.AddDistanceJoint(pivot_, bob_);
  world_.joint(rod_).length = kLength;
  measured_.reset();

  const Softness coefficients = softness();
  const Step hand = predicted();
  readouts_[0].value = ToDecimal(coefficients.bias_rate, readouts_[0].scale);
  readouts_[1].value = ToDecimal(coefficients.mass_scale, readouts_[1].scale);
  readouts_[2].value =
      ToDecimal(coefficients.impulse_scale, readouts_[2].scale);
  readouts_[3].value = ToDecimal(hand.impulse, readouts_[3].scale);
  readouts_[5].value = ToDecimal(hand.velocity, readouts_[5].scale);
  readouts_[7].value = ToDecimal(hand.error * Scalar{1000}, readouts_[7].scale);
  for (const std::size_t i : {0u, 1u, 2u, 3u, 5u, 7u}) {
    readouts_[i].is_valid = true;
  }
}

void OneStepByHandSample::FixedUpdate() {
  world_.Step();
  if (measured_.has_value()) {
    return;
  }
  measured_ = Step{.impulse = world_.joint(rod_).impulse,
                   .velocity = world_.body(bob_).velocity.y(),
                   .error = length_error()};
  readouts_[4].value = ToDecimal(measured_->impulse, readouts_[4].scale);
  readouts_[6].value = ToDecimal(measured_->velocity, readouts_[6].scale);
  readouts_[8].value =
      ToDecimal(measured_->error * Scalar{1000}, readouts_[8].scale);
  for (const std::size_t i : {4u, 6u, 8u}) {
    readouts_[i].is_valid = true;
  }
}

OneStepByHandSample::Softness OneStepByHandSample::softness() const {
  const Scalar h = world_.dt();
  const Scalar omega =
      Scalar{2} * Pi<Scalar>() * static_cast<Scalar>(parameters_[kHertz].value);
  const Scalar zeta = parameters_[kDamping].ToScalar();
  const Scalar a1 = Scalar{2} * zeta + h * omega;
  const Scalar a2 = h * omega * a1;
  const Scalar a3 = Scalar{1} / (Scalar{1} + a2);
  return {.bias_rate = omega / a1, .mass_scale = a2 * a3, .impulse_scale = a3};
}

OneStepByHandSample::Step OneStepByHandSample::predicted() const {
  const Scalar h = world_.dt();
  const Softness coefficients = softness();
  const Scalar error = Ratio(parameters_[kOffset].value, 1000);
  // Gravity comes first: the bob moves down, away from the pivot
  const Scalar velocity = -kGravity * h;
  const Scalar separation = -velocity;
  // k = 1/m = 1 per kg (the pivot is static); a cold first step leaks
  // nothing
  const Scalar impulse =
      coefficients.mass_scale * (-coefficients.bias_rate * error - separation);
  // The axis points down: +impulse along it is -impulse along y
  const Scalar velocity_after = velocity - impulse;
  return {.impulse = impulse,
          .velocity = velocity_after,
          .error = error - velocity_after * h};
}

Scalar OneStepByHandSample::length_error() const {
  return world_.JointDistance(rod_) - kLength;
}

void OneStepByHandSample::Draw(DrawList& draw_list) const {
  const Body& pivot = world_.body(pivot_);
  const Body& bob = world_.body(bob_);
  draw_list.lines.push_back(
      {pivot.position, bob.position, sample_color::kWhite});
  const Scalar rest = pivot.position.y() - kLength;
  draw_list.lines.push_back({Vec2f{pivot.position.x() - Ratio(1, 2), rest},
                             Vec2f{pivot.position.x() - Ratio(1, 5), rest},
                             sample_color::kGreen});
  draw_list.circles.push_back(
      {pivot.position, kPivotRadius, sample_color::kGrey});
  draw_list.circles.push_back(
      {bob.position, kBobRadius, sample_color::kOrange});
}

void OneStepByHandSample::End() {
  world_.Clear();
  measured_.reset();
  for (auto& readout : readouts_) {
    readout.is_valid = false;
  }
}

}  // namespace neko::tutorial
