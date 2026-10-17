#include "tutorials/07_pendulum/solutions/tension_sample.h"

#include "neko/math/angle.h"
#include "neko/math/func.h"

#include "main/samples/body_draw.h"

namespace neko::tutorial {
namespace {
constexpr Scalar kGravity = Ratio(981, 100);
// The gap is measured once the soft step took the load: after 0.5 s
constexpr int kSettleSteps = 30;
constexpr Scalar kBobRadius = Ratio(3, 20);
constexpr Scalar kPivotRadius = Ratio(1, 20);
}  // namespace

void TensionSample::Begin(const SampleContext& context) {
  const Vec2f top{context.world_bounds.GetCenter().x(),
                  context.world_bounds.max_bound().y() - Ratio(5, 2)};
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
  max_tension_ = Scalar{0};
  min_tension_ = Scalar{0};
  max_error_ = Scalar{0};
  step_ = 0;

  readouts_[0].value =
      ToDecimal(predicted_bottom_tension(), readouts_[0].scale);
  readouts_[0].is_valid = true;
}

void TensionSample::FixedUpdate() {
  world_.Step();
  ++step_;
  const Scalar now = tension();
  const Scalar predicted = predicted_tension();
  if (step_ == 1) {
    first_tension_ = now;
  }
  max_tension_ = Max(max_tension_, now);
  min_tension_ = Min(min_tension_, now);
  if (step_ > kSettleSteps) {
    max_error_ = Max(max_error_, Abs(now - predicted));
  }
  readouts_[1].value = ToDecimal(max_tension_, readouts_[1].scale);
  readouts_[2].value = ToDecimal(first_tension_, readouts_[2].scale);
  readouts_[3].value = ToDecimal(now, readouts_[3].scale);
  readouts_[4].value = ToDecimal(predicted, readouts_[4].scale);
  readouts_[5].value = ToDecimal(max_error_, readouts_[5].scale);
  for (std::size_t i = 1; i < readouts_.size(); ++i) {
    readouts_[i].is_valid = true;
  }
}

void TensionSample::Draw(DrawList& draw_list) const {
  const Body& pivot = world_.body(pivot_);
  const Body& bob = world_.body(bob_);
  draw_list.lines.push_back(
      {pivot.position, bob.position,
       tension() < Scalar{0} ? sample_color::kRed : sample_color::kWhite});
  draw_list.circles.push_back(
      {pivot.position, kPivotRadius, sample_color::kGrey});
  draw_list.circles.push_back(
      {bob.position, kBobRadius, sample_color::kOrange});
  DrawVelocity(bob, sample_color::kSky, draw_list);
}

void TensionSample::End() {
  world_.Clear();
  for (auto& readout : readouts_) {
    readout.is_valid = false;
  }
}

Scalar TensionSample::cos_amplitude() const {
  return Cos(
      Radian{Degree{static_cast<Scalar>(parameters_[kAmplitude].value)}});
}

Scalar TensionSample::predicted_bottom_tension() const {
  // A 1 kg bob
  return kGravity * (Scalar{3} - Scalar{2} * cos_amplitude());
}

Scalar TensionSample::tension() const {
  return -world_.joint(rod_).impulse / world_.dt();
}

Scalar TensionSample::predicted_tension() const {
  // cos theta from the bob's depth below the pivot, over the distance
  const Vec2f delta = world_.body(bob_).position - world_.body(pivot_).position;
  const Scalar cos_theta = -delta.y() / delta.Length();
  return kGravity * (Scalar{3} * cos_theta - Scalar{2} * cos_amplitude());
}

}  // namespace neko::tutorial
