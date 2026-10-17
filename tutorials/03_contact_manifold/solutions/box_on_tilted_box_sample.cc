#include "tutorials/03_contact_manifold/solutions/box_on_tilted_box_sample.h"

#include "neko/math/angle.h"
#include "neko/math/func.h"
#include "neko/math/rotation.h"

#include "main/samples/body_draw.h"

namespace neko::tutorial {
namespace {
constexpr Vec2f kRampHalfSize{Scalar{5}, Ratio(1, 4)};
constexpr Vec2f kBoxHalfSize{Scalar{1}, Ratio(1, 2)};
constexpr Scalar kPointRadius = Ratio(1, 16);
// A manifold point belongs to a corner when it is this close to it, in meters
constexpr Scalar kCornerMatch = Ratio(1, 100);
}  // namespace

void BoxOnTiltedBoxSample::Begin(const SampleContext& context) {
  const int ramp_angle = parameters_[kRampAngle].value;
  ramp_index_ = world_.AddBody();
  {
    Body& ramp = world_.body(ramp_index_);
    ramp.type = BodyType::kStatic;
    ramp.inverse_mass = Scalar{0};
    ramp.position =
        context.world_bounds.GetCenter() - Vec2f{Scalar{0}, Scalar{1}};
    ramp.rotation.set_angle(Radian{Degree{static_cast<Scalar>(ramp_angle)}});
    ramp_collider_ = world_.AddOrientedBoxCollider(ramp_index_);
    world_.collider(ramp_collider_).is_trigger = false;
    world_.oriented_box(world_.collider(ramp_collider_).shape_index).half_size =
        kRampHalfSize;
  }

  box_index_ = world_.AddBody();
  {
    const Body& ramp = world_.body(ramp_index_);
    const Scalar height = kRampHalfSize.y() + kBoxHalfSize.y() -
                          parameters_[kSinkDepth].ToScalar();
    Body& box = world_.body(box_index_);
    box.type = BodyType::kKinematic;
    box.position = ramp.position +
                   ramp.rotation.x_axis() * parameters_[kSlide].ToScalar() +
                   ramp.rotation.y_axis() * height;
    box.rotation.set_angle(Radian{Degree{
        static_cast<Scalar>(ramp_angle + parameters_[kRelativeTilt].value)}});
    box_collider_ = world_.AddOrientedBoxCollider(box_index_);
    world_.collider(box_collider_).is_trigger = false;
    world_.oriented_box(world_.collider(box_collider_).shape_index).half_size =
        kBoxHalfSize;
  }
  UpdateContact();
}

void BoxOnTiltedBoxSample::FixedUpdate() {
  world_.Step();
  UpdateContact();
}

void BoxOnTiltedBoxSample::UpdateContact() {
  // Ramp first (lower collider index): the normal points from the box toward
  // the ramp
  contact_ = Contact{};
  if (!world_.DetectContact(
          world_.body(ramp_index_), world_.collider(ramp_collider_),
          world_.body(box_index_), world_.collider(box_collider_), &contact_)) {
    contact_.point_count = 0;
  }

  readouts_[0].value = predicted_point_count();
  readouts_[1].value = contact_.point_count;
  readouts_[0].is_valid = true;
  readouts_[1].is_valid = true;
  for (std::size_t corner = 0; corner < 2; ++corner) {
    SampleReadout& predicted = readouts_[2 + 2 * corner];
    SampleReadout& measured = readouts_[3 + 2 * corner];
    const auto predicted_value = predicted_depth(corner);
    const auto measured_value = measured_depth(corner);
    predicted.is_valid = predicted_value.has_value();
    measured.is_valid = measured_value.has_value();
    if (predicted_value.has_value()) {
      predicted.value = ToDecimal(*predicted_value, predicted.scale);
    }
    if (measured_value.has_value()) {
      measured.value = ToDecimal(*measured_value, measured.scale);
    }
  }
  const Vec2f normal = predicted_normal();
  const bool touching = contact_.point_count > 0;
  readouts_[6].value = ToDecimal(normal.x(), readouts_[6].scale);
  readouts_[8].value = ToDecimal(normal.y(), readouts_[8].scale);
  readouts_[7].value =
      ToDecimal(contact_.contact_normal.x(), readouts_[7].scale);
  readouts_[9].value =
      ToDecimal(contact_.contact_normal.y(), readouts_[9].scale);
  readouts_[6].is_valid = true;
  readouts_[8].is_valid = true;
  readouts_[7].is_valid = touching;
  readouts_[9].is_valid = touching;
}

Vec2f BoxOnTiltedBoxSample::ToRampFrame(Vec2f point) const {
  const Body& ramp = world_.body(ramp_index_);
  const Vec2f delta = point - ramp.position;
  return {Vec2f::Dot(delta, ramp.rotation.x_axis()),
          Vec2f::Dot(delta, ramp.rotation.y_axis())};
}

std::array<Vec2f, 2> BoxOnTiltedBoxSample::local_corners() const {
  // Chapter 03's corners, with the relative tilt, from the box's own center
  // seen in the ramp's frame
  Rotation tilt;
  tilt.set_angle(
      Radian{Degree{static_cast<Scalar>(parameters_[kRelativeTilt].value)}});
  const Vec2f center = ToRampFrame(world_.body(box_index_).position);
  return {center + tilt.Rotate(-kBoxHalfSize),
          center + tilt.Rotate({kBoxHalfSize.x(), -kBoxHalfSize.y()})};
}

std::optional<Scalar> BoxOnTiltedBoxSample::predicted_depth(
    std::size_t corner) const {
  const Scalar depth = kRampHalfSize.y() - local_corners()[corner].y();
  if (depth < Scalar{0}) {
    return std::nullopt;
  }
  return depth;
}

int BoxOnTiltedBoxSample::predicted_point_count() const {
  return (predicted_depth(0).has_value() ? 1 : 0) +
         (predicted_depth(1).has_value() ? 1 : 0);
}

std::optional<Scalar> BoxOnTiltedBoxSample::measured_depth(
    std::size_t corner) const {
  const Vec2f position = local_corners()[corner];
  for (std::uint8_t i = 0; i < contact_.point_count; ++i) {
    const Vec2f offset = ToRampFrame(contact_.points[i].position) - position;
    if (Abs(offset.x()) < kCornerMatch && Abs(offset.y()) < kCornerMatch) {
      return contact_.points[i].penetration;
    }
  }
  return std::nullopt;
}

Vec2f BoxOnTiltedBoxSample::predicted_normal() const {
  return -world_.body(ramp_index_).rotation.y_axis();
}

void BoxOnTiltedBoxSample::Draw(DrawList& draw_list) const {
  const Body& ramp = world_.body(ramp_index_);
  const Body& box = world_.body(box_index_);
  DrawBoxOutline(ramp.position, kRampHalfSize, ramp.rotation,
                 sample_color::kGrey, draw_list);
  DrawBoxOutline(box.position, kBoxHalfSize, box.rotation,
                 sample_color::kOrange, draw_list);
  // The predicted corners back in the world
  for (std::size_t corner = 0; corner < 2; ++corner) {
    if (!predicted_depth(corner).has_value()) {
      continue;
    }
    const Vec2f local = local_corners()[corner];
    draw_list.circles.push_back(
        {ramp.position + ramp.rotation.x_axis() * local.x() +
             ramp.rotation.y_axis() * local.y(),
         kPointRadius * Scalar{2}, sample_color::kGreen});
  }
  for (std::uint8_t i = 0; i < contact_.point_count; ++i) {
    const ContactPoint& point = contact_.points[i];
    draw_list.circles.push_back(
        {point.position, kPointRadius, sample_color::kYellow});
    draw_list.lines.push_back(
        {point.position,
         point.position - contact_.contact_normal * point.penetration,
         sample_color::kRed});
  }
}

void BoxOnTiltedBoxSample::End() {
  world_.Clear();
  contact_ = Contact{};
}

}  // namespace neko::tutorial
