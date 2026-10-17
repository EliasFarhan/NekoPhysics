#include "tutorials/01_rotation/solutions/single_force_sample.h"

#include "main/samples/body_draw.h"

namespace neko::tutorial {
namespace {
constexpr Vec2f kHalfSize{Scalar{1}, Ratio(1, 2)};
// Forces drawn as arrows of this many meters per newton
constexpr Scalar kForceDrawScale = Ratio(1, 4);
// Steps per second of the push time parameter (dt = 1/60 s)
constexpr int kStepsPerSecond = 60;
}  // namespace

void SingleForceSample::Begin(const SampleContext& context) {
  const SampleParameter& push_time = parameters_[kPushTime];
  push_steps_ = push_time.value * kStepsPerSecond / push_time.scale;

  box_index_ = world_.AddBody();
  Body& box = world_.body(box_index_);
  // Low in the window: the push moves the center up
  box.position = context.world_bounds.GetCenter() - Vec2f{Scalar{0}, Scalar{2}};
  box.inverse_mass = Scalar{1} / parameters_[kMass].ToScalar();
  const ColliderIndex collider_index = world_.AddAabbCollider(box_index_);
  Collider& collider = world_.collider(collider_index);
  collider.is_trigger = false;
  world_.aabb(collider.shape_index).half_size = kHalfSize;

  readouts_[0].value = ToDecimal(predicted_speed(), readouts_[0].scale);
  readouts_[2].value = ToDecimal(predicted_spin(), readouts_[2].scale);
  for (SampleReadout& readout : readouts_) {
    readout.is_valid = true;
  }
}

void SingleForceSample::FixedUpdate() {
  if (is_pushing()) {
    // World-fixed lever arm and force: the torque stays r F
    const Body& box = world_.body(box_index_);
    const Vec2f lever{parameters_[kLeverArm].ToScalar(), Scalar{0}};
    const Vec2f force{Scalar{0}, parameters_[kForce].ToScalar()};
    world_.ApplyForceAtPoint(box_index_, force, box.position + lever);
  }
  world_.Step();
  ++step_;
  const Body& box = world_.body(box_index_);
  readouts_[1].value = ToDecimal(box.velocity.Length(), readouts_[1].scale);
  readouts_[3].value = ToDecimal(box.angular_velocity, readouts_[3].scale);
}

void SingleForceSample::Draw(DrawList& draw_list) const {
  const Body& box = world_.body(box_index_);
  draw_list.circles.push_back(
      {box.position, Ratio(1, 20), sample_color::kGrey});
  DrawBoxOutline(box.position, kHalfSize, box.rotation, sample_color::kOrange,
                 draw_list);
  DrawOrientation(box, kHalfSize.x(), sample_color::kWhite, draw_list);
  DrawVelocity(box, sample_color::kSky, draw_list);
  if (is_pushing()) {
    const Vec2f point =
        box.position + Vec2f{parameters_[kLeverArm].ToScalar(), Scalar{0}};
    const Vec2f arrow{Scalar{0},
                      parameters_[kForce].ToScalar() * kForceDrawScale};
    draw_list.lines.push_back({box.position, point, sample_color::kGrey});
    draw_list.lines.push_back({point, point + arrow, sample_color::kRed});
  }
}

void SingleForceSample::End() {
  world_.Clear();
  step_ = 0;
}

Scalar SingleForceSample::predicted_inertia() const {
  const Scalar width = Scalar{2} * kHalfSize.x();
  const Scalar height = Scalar{2} * kHalfSize.y();
  return parameters_[kMass].ToScalar() * (width * width + height * height) /
         Scalar{12};
}

Scalar SingleForceSample::push_time() const {
  return static_cast<Scalar>(push_steps_) * world_.dt();
}

Scalar SingleForceSample::predicted_speed() const {
  return parameters_[kForce].ToScalar() * push_time() /
         parameters_[kMass].ToScalar();
}

Scalar SingleForceSample::predicted_spin() const {
  return parameters_[kLeverArm].ToScalar() * parameters_[kForce].ToScalar() *
         push_time() / predicted_inertia();
}

}  // namespace neko::tutorial
