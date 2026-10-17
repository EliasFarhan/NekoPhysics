#include "tutorials/11_jetpack/solutions/terminal_velocity_sample.h"

#include "neko/math/func.h"

namespace neko::tutorial {
namespace {
constexpr Scalar kGravity = Ratio(981, 100);
// The graph: ten seconds over 10 m, the terminal velocity 5 m up
constexpr Scalar kGraphWidth{10};
constexpr Scalar kGraphHeight{5};
}  // namespace

void TerminalVelocitySample::Begin(const SampleContext& context) {
  player_ = world_.AddBody();
  Body& player = world_.body(player_);
  player.inverse_mass = Scalar{1} / mass();
  player.inertia_mode = InertiaMode::kFixedRotation;
  start_y_ = player.position.y();
  origin_ = context.world_bounds.min_bound() + Vec2f{Ratio(7, 5), Scalar{1}};
  step_ = 0;
  time_to_99_.reset();
  speeds_.fill(Scalar{0});
  readouts_[0].value = ToDecimal(predicted_terminal(), readouts_[0].scale);
  readouts_[0].is_valid = true;
}

Scalar TerminalVelocitySample::predicted_terminal() const {
  const Scalar k1 = parameters_[kLinear].ToScalar();
  const Scalar k2 = parameters_[kQuadratic].ToScalar();
  const Scalar weight = mass() * kGravity;
  if (k2 == Scalar{0}) {
    return k1 > Scalar{0} ? weight / k1 : Scalar{0};
  }
  // The positive root of k2 v^2 + k1 v - m g = 0
  return (Sqrt(k1 * k1 + Scalar{4} * k2 * weight) - k1) / (Scalar{2} * k2);
}

Scalar TerminalVelocitySample::speed() const {
  return world_.body(player_).velocity.Length();
}

Scalar TerminalVelocitySample::fallen() const {
  return start_y_ - world_.body(player_).position.y();
}

void TerminalVelocitySample::FixedUpdate() {
  if (step_ >= kMeasuredSteps) {
    return;
  }
  const Body& player = world_.body(player_);
  const Scalar k1 = parameters_[kLinear].ToScalar();
  const Scalar k2 = parameters_[kQuadratic].ToScalar();
  // Millington's drag: against the velocity, k1 |v| + k2 |v|^2
  world_.ApplyForce(player_, -player.velocity * (k1 + k2 * speed()));
  world_.Step();
  ++step_;
  const Scalar terminal = predicted_terminal();
  if (!time_to_99_.has_value() && terminal > Scalar{0} &&
      speed() >= terminal * Ratio(99, 100)) {
    time_to_99_ = static_cast<Scalar>(step_) * world_.dt();
    readouts_[2].value = ToDecimal(*time_to_99_, readouts_[2].scale);
    readouts_[2].is_valid = true;
  }
  if (step_ % kStepsPerPoint == 0) {
    speeds_[static_cast<std::size_t>(step_ / kStepsPerPoint)] = speed();
  }
  readouts_[1].value = ToDecimal(speed(), readouts_[1].scale);
  readouts_[3].value = ToDecimal(fallen(), readouts_[3].scale);
  readouts_[1].is_valid = true;
  readouts_[3].is_valid = true;
}

void TerminalVelocitySample::Draw(DrawList& draw_list) const {
  const Scalar terminal = predicted_terminal();
  const Scalar per_speed =
      terminal > Scalar{0} ? kGraphHeight / terminal : Scalar{0};
  draw_list.lines.push_back(
      {origin_, origin_ + Vec2f{kGraphWidth, Scalar{0}}, sample_color::kGrey});
  draw_list.lines.push_back(
      {origin_, origin_ + Vec2f{Scalar{0}, kGraphHeight * Ratio(6, 5)},
       sample_color::kGrey});
  draw_list.lines.push_back({origin_ + Vec2f{Scalar{0}, kGraphHeight},
                             origin_ + Vec2f{kGraphWidth, kGraphHeight},
                             sample_color::kGreen});
  const std::size_t shown =
      static_cast<std::size_t>(step_ / kStepsPerPoint) + 1;
  const Scalar per_point = kGraphWidth / static_cast<Scalar>(kPoints - 1);
  for (std::size_t i = 1; i < shown; ++i) {
    const Vec2f from{origin_.x() + per_point * static_cast<Scalar>(i - 1),
                     origin_.y() + Min(speeds_[i - 1] * per_speed,
                                       kGraphHeight * Ratio(6, 5))};
    const Vec2f to{
        origin_.x() + per_point * static_cast<Scalar>(i),
        origin_.y() + Min(speeds_[i] * per_speed, kGraphHeight * Ratio(6, 5))};
    draw_list.lines.push_back({from, to, sample_color::kWhite});
  }
}

void TerminalVelocitySample::End() {
  world_.Clear();
  step_ = 0;
  time_to_99_.reset();
  for (auto& readout : readouts_) {
    readout.is_valid = false;
  }
}

}  // namespace neko::tutorial
