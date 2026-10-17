#include "tutorials/01_rotation/solutions/dumbbell_sample.h"

#include "main/samples/body_draw.h"

namespace neko::tutorial {
namespace {
// Forces drawn as arrows of this many meters per newton
constexpr Scalar kForceDrawScale = Ratio(1, 4);
// Steps per second of the push time parameter (dt = 1/60 s)
constexpr int kStepsPerSecond = 60;
}  // namespace

void DumbbellSample::Begin(const SampleContext& context) {
  const SampleParameter& push_time = parameters_[kPushTime];
  push_steps_ = push_time.value * kStepsPerSecond / push_time.scale;

  body_index_ = world_.AddBody();
  Body& body = world_.body(body_index_);
  body.position = context.world_bounds.GetCenter();
  body.inverse_mass = Scalar{1} / parameters_[kMass].ToScalar();
  const Scalar half_side = parameters_[kSide].ToScalar() / Scalar{2};
  const Scalar offset = parameters_[kOffset].ToScalar();
  for (const Scalar side : {-offset, offset}) {
    const ColliderIndex collider_index = world_.AddAabbCollider(body_index_);
    Collider& collider = world_.collider(collider_index);
    collider.is_trigger = false;
    collider.offset = Vec2f{side, Scalar{0}};
    world_.aabb(collider.shape_index).half_size = Vec2f{half_side, half_side};
  }

  readouts_[0].value = ToDecimal(predicted_inertia(), readouts_[0].scale);
  readouts_[1].value = ToDecimal(offset_inertia(), readouts_[1].scale);
  readouts_[2].value = ToDecimal(measured_inertia(), readouts_[2].scale);
  readouts_[3].value = ToDecimal(predicted_spin(), readouts_[3].scale);
  for (SampleReadout& readout : readouts_) {
    readout.is_valid = true;
  }
}

void DumbbellSample::FixedUpdate() {
  if (is_pushing()) {
    // The couple of chapter 01: it turns with the body, torque 2 r F
    const Body& body = world_.body(body_index_);
    const Vec2f lever =
        body.rotation.x_axis() * parameters_[kLeverArm].ToScalar();
    const Vec2f force = body.rotation.y_axis() * parameters_[kForce].ToScalar();
    world_.ApplyForceAtPoint(body_index_, force, body.position + lever);
    world_.ApplyForceAtPoint(body_index_, -force, body.position - lever);
  }
  world_.Step();
  ++step_;
  readouts_[4].value =
      ToDecimal(world_.body(body_index_).angular_velocity, readouts_[4].scale);
}

void DumbbellSample::Draw(DrawList& draw_list) const {
  const Body& body = world_.body(body_index_);
  const Scalar half_side = parameters_[kSide].ToScalar() / Scalar{2};
  const Vec2f to_box = body.rotation.x_axis() * parameters_[kOffset].ToScalar();
  draw_list.lines.push_back(
      {body.position - to_box, body.position + to_box, sample_color::kGrey});
  for (const Vec2f& center : {body.position - to_box, body.position + to_box}) {
    DrawBoxOutline(center, Vec2f{half_side, half_side}, body.rotation,
                   sample_color::kOrange, draw_list);
  }
  DrawOrientation(body, half_side, sample_color::kWhite, draw_list);
  if (is_pushing()) {
    const Vec2f lever =
        body.rotation.x_axis() * parameters_[kLeverArm].ToScalar();
    const Vec2f arrow = body.rotation.y_axis() *
                        (parameters_[kForce].ToScalar() * kForceDrawScale);
    for (const Vec2f& side : {lever, -lever}) {
      const Vec2f point = body.position + side;
      const Vec2f push = side == lever ? arrow : -arrow;
      draw_list.lines.push_back({point, point + push, sample_color::kRed});
    }
  }
}

void DumbbellSample::End() {
  world_.Clear();
  step_ = 0;
}

Scalar DumbbellSample::predicted_inertia() const {
  const Scalar side = parameters_[kSide].ToScalar();
  return parameters_[kMass].ToScalar() * side * side / Scalar{6} +
         offset_inertia();
}

Scalar DumbbellSample::offset_inertia() const {
  const Scalar offset = parameters_[kOffset].ToScalar();
  return parameters_[kMass].ToScalar() * offset * offset;
}

Scalar DumbbellSample::measured_inertia() const {
  return Scalar{1} / world_.ComputeInverseInertia(body_index_);
}

Scalar DumbbellSample::push_time() const {
  return static_cast<Scalar>(push_steps_) * world_.dt();
}

Scalar DumbbellSample::predicted_spin() const {
  return Scalar{2} * parameters_[kLeverArm].ToScalar() *
         parameters_[kForce].ToScalar() * push_time() / predicted_inertia();
}

}  // namespace neko::tutorial
