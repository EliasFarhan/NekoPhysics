#include "tutorials/07_pendulum/solutions/where_slack_sample.h"

#include "neko/math/func.h"

#include "main/samples/body_draw.h"

namespace neko::tutorial {
namespace {
constexpr Scalar kGravity = Ratio(981, 100);
constexpr Scalar kBobRadius = Ratio(3, 20);
constexpr Scalar kPivotRadius = Ratio(1, 20);
// A rope shorter than its length by more than this is slack
constexpr Scalar kSlackTolerance = Ratio(1, 100);
}  // namespace

void WhereSlackSample::Begin(const SampleContext& context) {
  const Vec2f center = context.world_bounds.GetCenter();
  pivot_ = world_.AddBody();
  world_.body(pivot_).type = BodyType::kStatic;
  world_.body(pivot_).inverse_mass = Scalar{0};
  world_.body(pivot_).position = center;

  // A point mass at the bottom, launched to the right
  bob_ = world_.AddBody();
  world_.body(bob_).inverse_mass = Scalar{1};
  world_.body(bob_).position = center + Vec2f{Scalar{0}, -length()};
  world_.body(bob_).velocity = {speed(), Scalar{0}};

  rope_ = world_.AddDistanceJoint(pivot_, bob_);
  world_.joint(rope_).length = length();
  world_.joint(rope_).type = DistanceJointType::kRope;

  readouts_[0].value = ToDecimal(predicted_slack_height(), readouts_[0].scale);
  readouts_[0].is_valid = true;
  readouts_[2].is_valid = true;
}

void WhereSlackSample::FixedUpdate() {
  world_.Step();
  const Scalar height =
      world_.body(bob_).position.y() - world_.body(pivot_).position.y();
  // The rope's impulse is 0 in a step where it didn't need to pull
  if (!slack_height_.has_value() && world_.joint(rope_).impulse == Scalar{0}) {
    slack_height_ = height;
    readouts_[1].value = ToDecimal(height, readouts_[1].scale);
    readouts_[1].is_valid = true;
  }
  if (world_.JointDistance(rope_) < length() - kSlackTolerance) {
    ++slack_steps_;
  }
  readouts_[2].value = slack_steps_;
}

void WhereSlackSample::Draw(DrawList& draw_list) const {
  const Body& pivot = world_.body(pivot_);
  const Body& bob = world_.body(bob_);
  const bool is_slack =
      world_.JointDistance(rope_) < length() - kSlackTolerance;
  draw_list.lines.push_back(
      {pivot.position, bob.position,
       is_slack ? sample_color::kDarkGrey : sample_color::kYellow});
  const Scalar slack_y = pivot.position.y() + predicted_slack_height();
  draw_list.lines.push_back({Vec2f{pivot.position.x() - length(), slack_y},
                             Vec2f{pivot.position.x() + length(), slack_y},
                             sample_color::kGreen});
  draw_list.circles.push_back(
      {pivot.position, kPivotRadius, sample_color::kGrey});
  draw_list.circles.push_back(
      {bob.position, kBobRadius, sample_color::kOrange});
  DrawVelocity(bob, sample_color::kSky, draw_list);
}

void WhereSlackSample::End() {
  world_.Clear();
  slack_height_.reset();
  slack_steps_ = 0;
  for (auto& readout : readouts_) {
    readout.is_valid = false;
  }
}

Scalar WhereSlackSample::predicted_slack_height() const {
  return (speed() * speed() - Scalar{2} * kGravity * length()) /
         (Scalar{3} * kGravity);
}

}  // namespace neko::tutorial
