#include "tutorials/01_rotation/rotation_sample.h"

#include "neko/math/angle.h"
#include "neko/math/rotation.h"

#include "main/samples/body_draw.h"

namespace neko::tutorial {
namespace {
// Forces drawn as arrows of this many meters per newton
constexpr Scalar kForceDrawScale = Ratio(1, 4);
// Steps per second of the push time parameter (dt = 1/60 s)
constexpr int kStepsPerSecond = 60;
}  // namespace

void RotationSample::Begin(const SampleContext& context) {
  const SampleParameter& push_time = parameters_[kPushTime];
  push_steps_ = push_time.value * kStepsPerSecond / push_time.scale;

  box_index_ = world_.AddBody();
  Body& box = world_.body(box_index_);
  box.position = context.world_bounds.GetCenter();
  box.inverse_mass = Scalar{1} / parameters_[kMass].ToScalar();
  // Solid: triggers have no mass, so no inertia
  const ColliderIndex collider_index = world_.AddAabbCollider(box_index_);
  Collider& collider = world_.collider(collider_index);
  collider.is_trigger = false;
  world_.aabb(collider.shape_index).half_size = half_size();

  readouts_[0].value = ToDecimal(predicted_inertia(), readouts_[0].scale);
  readouts_[1].value = ToDecimal(torque(), readouts_[1].scale);
  readouts_[2].value = ToDecimal(predicted_spin(), readouts_[2].scale);
  readouts_[4].value = ToDecimal(predicted_turn(), readouts_[4].scale);
  for (std::size_t i = 0; i < 5; ++i) {
    readouts_[i].is_valid = true;
  }
}

void RotationSample::FixedUpdate() {
  if (is_pushing()) {
    // The couple turns with the box: each force is perpendicular to its
    // lever arm, so the torque stays 2 r F
    const Body& box = world_.body(box_index_);
    const Vec2f lever =
        box.rotation.x_axis() * parameters_[kLeverArm].ToScalar();
    const Vec2f force = box.rotation.y_axis() * parameters_[kForce].ToScalar();
    world_.ApplyForceAtPoint(box_index_, force, box.position + lever);
    world_.ApplyForceAtPoint(box_index_, -force, box.position - lever);
  }
  world_.Step();
  ++step_;

  const Scalar angle = world_.body(box_index_).rotation.angle().value();
  turned_ += WrapAngle(Radian{angle - previous_angle_}).value();
  previous_angle_ = angle;
  if (step_ == push_steps_) {
    measured_turn_ = turned_;
    readouts_[5].value = ToDecimal(turned_, readouts_[5].scale);
    readouts_[5].is_valid = true;
  }
  readouts_[3].value = ToDecimal(spin(), readouts_[3].scale);
}

void RotationSample::Draw(DrawList& draw_list) const {
  const Body& box = world_.body(box_index_);
  const Vec2f half = half_size();
  draw_list.circles.push_back(
      {box.position, Ratio(1, 20), sample_color::kGrey});
  DrawBoxOutline(box.position, half, box.rotation, sample_color::kOrange,
                 draw_list);
  DrawOrientation(box, half.x(), sample_color::kWhite, draw_list);
  DrawVelocity(box, sample_color::kSky, draw_list);

  if (is_pushing()) {
    const Vec2f lever =
        box.rotation.x_axis() * parameters_[kLeverArm].ToScalar();
    const Vec2f arrow = box.rotation.y_axis() *
                        (parameters_[kForce].ToScalar() * kForceDrawScale);
    for (const Vec2f& side : {lever, -lever}) {
      const Vec2f point = box.position + side;
      draw_list.lines.push_back({box.position, point, sample_color::kGrey});
      // The force on the -lever side points the other way
      const Vec2f push = side == lever ? arrow : -arrow;
      draw_list.lines.push_back({point, point + push, sample_color::kRed});
    }
  }
  if (measured_turn_.has_value()) {
    const Scalar length = half.x() + Ratio(1, 2);
    const auto draw_direction = [&](Scalar turn, ColorId color) {
      const Rotation direction{Radian{turn}};
      draw_list.lines.push_back(
          {box.position, box.position + direction.x_axis() * length, color});
    };
    draw_direction(predicted_turn(), sample_color::kGreen);
    draw_direction(*measured_turn_, sample_color::kYellow);
  }
}

void RotationSample::End() {
  world_.Clear();
  step_ = 0;
  turned_ = Scalar{0};
  previous_angle_ = Scalar{0};
  measured_turn_.reset();
  readouts_[5].is_valid = false;
}

Scalar RotationSample::predicted_inertia() const {
  const Scalar width = parameters_[kWidth].ToScalar();
  const Scalar height = parameters_[kHeight].ToScalar();
  return parameters_[kMass].ToScalar() * (width * width + height * height) /
         Scalar{12};
}

Scalar RotationSample::measured_inertia() const {
  return Scalar{1} / world_.ComputeInverseInertia(box_index_);
}

Scalar RotationSample::torque() const {
  return Scalar{2} * parameters_[kLeverArm].ToScalar() *
         parameters_[kForce].ToScalar();
}

Scalar RotationSample::push_time() const {
  return static_cast<Scalar>(push_steps_) * world_.dt();
}

Scalar RotationSample::predicted_spin() const {
  return torque() / predicted_inertia() * push_time();
}

Scalar RotationSample::predicted_turn() const {
  const Scalar time = push_time();
  return torque() / predicted_inertia() * time * time / Scalar{2};
}

Scalar RotationSample::spin() const {
  return world_.body(box_index_).angular_velocity;
}

Vec2f RotationSample::half_size() const {
  return {parameters_[kWidth].ToScalar() / Scalar{2},
          parameters_[kHeight].ToScalar() / Scalar{2}};
}

}  // namespace neko::tutorial
