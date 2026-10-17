#include "tutorials/02_rotated_box/solutions/box_on_ramp_sample.h"

#include "neko/math/angle.h"
#include "neko/math/func.h"
#include "neko/math/rotation.h"

#include "main/samples/body_draw.h"

namespace neko::tutorial {
namespace {
constexpr Vec2f kRampHalfSize{Scalar{5}, Ratio(1, 4)};
constexpr Vec2f kBoxHalfSize{Ratio(1, 2), Ratio(1, 4)};
// Directions drawn this long, in meters
constexpr Scalar kArrowLength = Scalar{1};

// Half the length of a box's shadow on the unit axis
Scalar ShadowRadius(const Rotation& rotation, Vec2f half_size, Vec2f axis) {
  return half_size.x() * Abs(Vec2f::Dot(axis, rotation.x_axis())) +
         half_size.y() * Abs(Vec2f::Dot(axis, rotation.y_axis()));
}
}  // namespace

void BoxOnRampSample::Begin(const SampleContext& context) {
  const auto add_box = [this](BodyIndex& index, int degrees, Vec2f half_size) {
    index = world_.AddBody();
    world_.body(index).rotation.set_angle(
        Radian{Degree{static_cast<Scalar>(degrees)}});
    const ColliderIndex collider_index = world_.AddOrientedBoxCollider(index);
    Collider& collider = world_.collider(collider_index);
    collider.is_trigger = false;
    // Frictionless and without bounce: the impulse is along the normal
    collider.restitution = Scalar{0};
    collider.friction = Scalar{0};
    world_.oriented_box(collider.shape_index).half_size = half_size;
  };
  add_box(ramp_index_, parameters_[kRampAngle].value, kRampHalfSize);
  {
    Body& ramp = world_.body(ramp_index_);
    ramp.type = BodyType::kStatic;
    ramp.inverse_mass = Scalar{0};
    ramp.position =
        context.world_bounds.GetCenter() - Vec2f{Scalar{0}, Ratio(3, 2)};
  }
  add_box(box_index_, parameters_[kBoxAngle].value, kBoxHalfSize);
  Body& box = world_.body(box_index_);
  box.inverse_mass = Scalar{1};
  box.position = world_.body(ramp_index_).position +
                 Vec2f{Scalar{0}, parameters_[kDropHeight].ToScalar()};
}

void BoxOnRampSample::FixedUpdate() {
  // The world tests contacts with the state of the start of the step
  const Body box_before = world_.body(box_index_);
  world_.Step();
  if (overlaps_.has_value() || world_.touching_pairs().empty()) {
    return;
  }

  // The separating axis test by hand, in the order of the world: the ramp's
  // axes, then the box's
  const Body& ramp = world_.body(ramp_index_);
  const std::array<Vec2f, 4> axes = {
      ramp.rotation.x_axis(), ramp.rotation.y_axis(),
      box_before.rotation.x_axis(), box_before.rotation.y_axis()};
  const Vec2f between = box_before.position - ramp.position;
  std::array<Scalar, 4> overlaps{};
  for (std::size_t i = 0; i < axes.size(); ++i) {
    overlaps[i] = ShadowRadius(ramp.rotation, kRampHalfSize, axes[i]) +
                  ShadowRadius(box_before.rotation, kBoxHalfSize, axes[i]) -
                  Abs(Vec2f::Dot(between, axes[i]));
    readouts_[i].value = ToDecimal(overlaps[i], readouts_[i].scale);
    readouts_[i].is_valid = true;
  }
  overlaps_ = overlaps;
  best_axis_ = 0;
  for (int i = 1; i < 4; ++i) {
    if (overlaps[static_cast<std::size_t>(i)] <
        overlaps[static_cast<std::size_t>(best_axis_)]) {
      best_axis_ = i;
    }
  }
  const Vec2f axis = axes[static_cast<std::size_t>(best_axis_)];
  predicted_normal_ = Vec2f::Dot(between, axis) < Scalar{0} ? -axis : axis;
  readouts_[4].value = ToDecimal(predicted_normal_->x(), readouts_[4].scale);
  readouts_[5].value = ToDecimal(predicted_normal_->y(), readouts_[5].scale);
  readouts_[4].is_valid = true;
  readouts_[5].is_valid = true;

  const Vec2f impact = box_before.velocity + world_.gravity() * world_.dt();
  const Vec2f change = world_.body(box_index_).velocity - impact;
  if (change != Vec2f::Zero()) {
    measured_normal_ = change * (Scalar{1} / change.Length());
    readouts_[6].value = ToDecimal(measured_normal_->x(), readouts_[6].scale);
    readouts_[7].value = ToDecimal(measured_normal_->y(), readouts_[7].scale);
    readouts_[6].is_valid = true;
    readouts_[7].is_valid = true;
  }
}

void BoxOnRampSample::Draw(DrawList& draw_list) const {
  const Body& ramp = world_.body(ramp_index_);
  const Body& box = world_.body(box_index_);
  DrawBoxOutline(ramp.position, kRampHalfSize, ramp.rotation,
                 sample_color::kGrey, draw_list);
  DrawBoxOutline(box.position, kBoxHalfSize, box.rotation,
                 sample_color::kOrange, draw_list);
  DrawOrientation(box, kBoxHalfSize.x(), sample_color::kWhite, draw_list);
  if (predicted_normal_.has_value() && measured_normal_.has_value()) {
    draw_list.lines.push_back({box.position,
                               box.position + *predicted_normal_ * kArrowLength,
                               sample_color::kGreen});
    draw_list.lines.push_back({box.position,
                               box.position + *measured_normal_ * kArrowLength,
                               sample_color::kYellow});
  }
}

void BoxOnRampSample::End() {
  world_.Clear();
  overlaps_.reset();
  best_axis_ = -1;
  predicted_normal_.reset();
  measured_normal_.reset();
  for (SampleReadout& readout : readouts_) {
    readout.is_valid = false;
  }
}

Vec2f BoxOnRampSample::ramp_normal() const {
  return world_.body(ramp_index_).rotation.y_axis();
}

}  // namespace neko::tutorial
