#include "tutorials/01_rotation/solutions/half_step_sample.h"

#include "neko/math/angle.h"
#include "neko/math/rotation.h"

#include "main/samples/body_draw.h"

namespace neko::tutorial {
namespace {
constexpr Vec2f kHalfSize{Scalar{1}, Ratio(1, 2)};
// Forces drawn as arrows of this many meters per newton
constexpr Scalar kForceDrawScale = Ratio(1, 4);
// Frames per second of the push time parameter
constexpr int kFramesPerSecond = 60;
}  // namespace

void HalfStepSample::Begin(const SampleContext& context) {
  const SampleParameter& push_time = parameters_[kPushTime];
  push_frames_ = push_time.value * kFramesPerSecond / push_time.scale;

  world_.emplace(PhysicsWorldConfig{
      .gravity = Vec2f::Zero(),
      .dt = kFixedDeltaTime / static_cast<Scalar>(parameters_[kSubsteps].value),
      .max_bodies = 1,
      .max_colliders = 1,
      .max_touching_pairs = 1,
      .max_broadphase_pairs = 1});
  box_index_ = world_->AddBody();
  Body& box = world_->body(box_index_);
  box.position = context.world_bounds.GetCenter();
  box.inverse_mass = Scalar{1} / parameters_[kMass].ToScalar();
  const ColliderIndex collider_index = world_->AddAabbCollider(box_index_);
  Collider& collider = world_->collider(collider_index);
  collider.is_trigger = false;
  world_->aabb(collider.shape_index).half_size = kHalfSize;

  readouts_[0].value = ToDecimal(dt() * Scalar{1000}, readouts_[0].scale);
  readouts_[1].value = ToDecimal(predicted_turn(), readouts_[1].scale);
  readouts_[4].value = ToDecimal(predicted_excess(), readouts_[4].scale);
  readouts_[0].is_valid = true;
  readouts_[1].is_valid = true;
  readouts_[4].is_valid = true;
}

void HalfStepSample::Push() {
  const Body& box = world_->body(box_index_);
  const Vec2f lever = box.rotation.x_axis() * parameters_[kLeverArm].ToScalar();
  const Vec2f force = box.rotation.y_axis() * parameters_[kForce].ToScalar();
  world_->ApplyForceAtPoint(box_index_, force, box.position + lever);
  world_->ApplyForceAtPoint(box_index_, -force, box.position - lever);
}

void HalfStepSample::FixedUpdate() {
  const bool pushing = is_pushing();
  for (int substep = 0; substep < parameters_[kSubsteps].value; ++substep) {
    if (pushing) {
      Push();
    }
    world_->Step();
    const Scalar angle = world_->body(box_index_).rotation.angle().value();
    turned_ += WrapAngle(Radian{angle - previous_angle_}).value();
    previous_angle_ = angle;
  }
  ++frame_;
  if (frame_ == push_frames_) {
    measured_turn_ = turned_;
    readouts_[2].value = ToDecimal(turned_, readouts_[2].scale);
    readouts_[3].value =
        ToDecimal(turned_ - predicted_turn(), readouts_[3].scale);
    readouts_[2].is_valid = true;
    readouts_[3].is_valid = true;
  }
}

void HalfStepSample::Draw(DrawList& draw_list) const {
  const Body& box = world_->body(box_index_);
  draw_list.circles.push_back(
      {box.position, Ratio(1, 20), sample_color::kGrey});
  DrawBoxOutline(box.position, kHalfSize, box.rotation, sample_color::kOrange,
                 draw_list);
  DrawOrientation(box, kHalfSize.x(), sample_color::kWhite, draw_list);
  if (is_pushing()) {
    const Vec2f lever =
        box.rotation.x_axis() * parameters_[kLeverArm].ToScalar();
    const Vec2f arrow = box.rotation.y_axis() *
                        (parameters_[kForce].ToScalar() * kForceDrawScale);
    for (const Vec2f& side : {lever, -lever}) {
      const Vec2f point = box.position + side;
      draw_list.lines.push_back({box.position, point, sample_color::kGrey});
      const Vec2f push = side == lever ? arrow : -arrow;
      draw_list.lines.push_back({point, point + push, sample_color::kRed});
    }
  }
  if (measured_turn_.has_value()) {
    const Scalar length = kHalfSize.x() + Ratio(1, 2);
    const auto draw_direction = [&](Scalar turn, ColorId color) {
      const Rotation direction{Radian{turn}};
      draw_list.lines.push_back(
          {box.position, box.position + direction.x_axis() * length, color});
    };
    draw_direction(predicted_turn(), sample_color::kGreen);
    draw_direction(*measured_turn_, sample_color::kYellow);
  }
}

void HalfStepSample::End() {
  world_.reset();
  frame_ = 0;
  turned_ = Scalar{0};
  previous_angle_ = Scalar{0};
  measured_turn_.reset();
  readouts_[2].is_valid = false;
  readouts_[3].is_valid = false;
}

std::uint32_t HalfStepSample::GenerateChecksum() const {
  return world_.has_value() ? world_->GenerateChecksum() : 0;
}

Scalar HalfStepSample::dt() const {
  return kFixedDeltaTime / static_cast<Scalar>(parameters_[kSubsteps].value);
}

Scalar HalfStepSample::angular_acceleration() const {
  const Scalar width = Scalar{2} * kHalfSize.x();
  const Scalar height = Scalar{2} * kHalfSize.y();
  const Scalar inertia = parameters_[kMass].ToScalar() *
                         (width * width + height * height) / Scalar{12};
  return Scalar{2} * parameters_[kLeverArm].ToScalar() *
         parameters_[kForce].ToScalar() / inertia;
}

Scalar HalfStepSample::push_time() const {
  return static_cast<Scalar>(push_frames_) * kFixedDeltaTime;
}

Scalar HalfStepSample::predicted_turn() const {
  const Scalar time = push_time();
  return angular_acceleration() * time * time / Scalar{2};
}

Scalar HalfStepSample::predicted_excess() const {
  return angular_acceleration() * push_time() * dt() / Scalar{2};
}

Scalar HalfStepSample::spin() const {
  return world_->body(box_index_).angular_velocity;
}

}  // namespace neko::tutorial
