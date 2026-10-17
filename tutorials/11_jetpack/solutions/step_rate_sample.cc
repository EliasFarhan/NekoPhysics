#include "tutorials/11_jetpack/solutions/step_rate_sample.h"

#include "main/samples/body_draw.h"

namespace neko::tutorial {
namespace {
constexpr Scalar kRadius = Ratio(1, 5);
// Frames of the host per second
constexpr int kFrameRate = 60;
}  // namespace

void StepRateSample::Begin(const SampleContext& context) {
  // dt = 1/rate s, from the host's 1/60 s: exactly 2184, 1092, 728 and 546
  // units of 1/65536 s in Fixed16
  world_ =
      PhysicsWorld{{.gravity = Vec2f::Zero(),
                    .dt = kFixedDeltaTime * static_cast<Scalar>(kFrameRate) /
                          static_cast<Scalar>(rate()),
                    .max_bodies = 4,
                    .max_colliders = 1,
                    .max_touching_pairs = 1,
                    .max_broadphase_pairs = 1,
                    .max_joints = 0}};
  const Scalar top = context.world_bounds.max_bound().y();
  for (std::size_t i = 0; i < kBodyCount; ++i) {
    bodies_[i] = world_.AddBody();
    Body& body = world_.body(bodies_[i]);
    body.inverse_mass = Scalar{1};
    body.inertia_mode = InertiaMode::kFixedRotation;
    body.position = {kStartX,
                     top - Scalar{1} - Ratio(3, 2) * static_cast<Scalar>(i)};
  }
  frame_ = 0;
  steps_ = 0;
}

void StepRateSample::StepOnce() {
  const Scalar thrust = parameters_[kThrust].ToScalar();
  const Scalar impulse = parameters_[kImpulse].ToScalar();
  world_.ApplyForce(bodies_[kThrustBody], {thrust, Scalar{0}});
  if (steps_ == 0) {
    world_.ApplyImpulse(bodies_[kImpulseBody], {impulse, Scalar{0}});
    // Splash's burst: a force for one step only
    world_.ApplyForce(bodies_[kBurstBody],
                      {parameters_[kBurst].ToScalar(), Scalar{0}});
  }
  // The frame-dependent pattern: an impulse every step
  world_.ApplyImpulse(bodies_[kEveryStepBody], {impulse, Scalar{0}});
  world_.Step();
  ++steps_;
}

void StepRateSample::FixedUpdate() {
  ++frame_;
  // World steps due after this frame: frame * rate / 60
  const int due = frame_ * rate() / kFrameRate;
  while (steps_ < due && !done()) {
    StepOnce();
  }
  readouts_[0].value = ToDecimal(velocity(kThrustBody).x(), readouts_[0].scale);
  readouts_[1].value = ToDecimal(distance(kThrustBody), readouts_[1].scale);
  readouts_[2].value =
      ToDecimal(velocity(kImpulseBody).x(), readouts_[2].scale);
  readouts_[3].value = ToDecimal(velocity(kBurstBody).x(), readouts_[3].scale);
  readouts_[4].value =
      ToDecimal(velocity(kEveryStepBody).x(), readouts_[4].scale);
  readouts_[5].value = steps_;
  for (auto& readout : readouts_) {
    readout.is_valid = true;
  }
}

void StepRateSample::Draw(DrawList& draw_list) const {
  constexpr std::array<ColorId, kBodyCount> colors{
      sample_color::kOrange, sample_color::kGreen, sample_color::kRed,
      sample_color::kMagenta};
  for (std::size_t i = 0; i < kBodyCount; ++i) {
    const Body& body = world_.body(bodies_[i]);
    draw_list.circles.push_back({body.position, kRadius, colors[i]});
    DrawVelocity(body, sample_color::kWhite, draw_list);
  }
}

void StepRateSample::End() {
  world_.Clear();
  frame_ = 0;
  steps_ = 0;
  for (auto& readout : readouts_) {
    readout.is_valid = false;
  }
}

}  // namespace neko::tutorial
