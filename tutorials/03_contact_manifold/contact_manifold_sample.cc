#include "tutorials/03_contact_manifold/contact_manifold_sample.h"

#include "neko/math/angle.h"

#include "main/samples/body_draw.h"

namespace neko::tutorial {
namespace {
constexpr Vec2f kGroundHalfSize{Scalar{6}, Ratio(1, 2)};
constexpr Scalar kBoxHalfHeight = Ratio(1, 2);
constexpr Scalar kPointRadius = Ratio(1, 16);
// A manifold point belongs to a corner when it is this close to it, in meters
constexpr Scalar kCornerMatch = Ratio(1, 100);
}  // namespace

void ContactManifoldSample::Begin(const SampleContext& context) {
  const Vec2f center = context.world_bounds.GetCenter();

  ground_index_ = world_.AddBody();
  {
    Body& ground = world_.body(ground_index_);
    ground.type = BodyType::kStatic;
    ground.inverse_mass = Scalar{0};
    ground.position = center - Vec2f{Scalar{0}, Scalar{2}};
    ground_collider_ = world_.AddAabbCollider(ground_index_);
    world_.collider(ground_collider_).is_trigger = false;
    world_.aabb(world_.collider(ground_collider_).shape_index).half_size =
        kGroundHalfSize;
  }

  box_index_ = world_.AddBody();
  {
    Body& box = world_.body(box_index_);
    box.type = BodyType::kKinematic;
    box.position = {center.x(), ground_top() + kBoxHalfHeight -
                                    parameters_[kSinkDepth].ToScalar()};
    box.rotation.set_angle(
        Radian{Degree{static_cast<Scalar>(parameters_[kAngle].value)}});
    box.angular_velocity =
        Radian{Degree{static_cast<Scalar>(parameters_[kSpin].value)}}.value();
    box_collider_ = world_.AddOrientedBoxCollider(box_index_);
    world_.collider(box_collider_).is_trigger = false;
    world_.oriented_box(world_.collider(box_collider_).shape_index).half_size =
        {parameters_[kWidth].ToScalar() / Scalar{2}, kBoxHalfHeight};
  }
  UpdateContact();
}

void ContactManifoldSample::FixedUpdate() {
  world_.Step();
  UpdateContact();
}

void ContactManifoldSample::UpdateContact() {
  // The same call as the narrowphase, ground first (lower collider index):
  // the normal points from the box toward the ground
  contact_ = Contact{};
  Collider& ground = world_.collider(ground_collider_);
  Collider& box = world_.collider(box_collider_);
  if (!world_.DetectContact(world_.body(ground_index_), ground,
                            world_.body(box_index_), box, &contact_)) {
    contact_.point_count = 0;
  }

  readouts_[0].value = predicted_point_count();
  readouts_[1].value = measured_point_count();
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
}

void ContactManifoldSample::Draw(DrawList& draw_list) const {
  const Body& ground = world_.body(ground_index_);
  const Body& box = world_.body(box_index_);
  draw_list.boxes.push_back(
      {ground.position, kGroundHalfSize, sample_color::kDarkGrey});
  const Vec2f half_size =
      world_.oriented_box(world_.collider(box_collider_).shape_index).half_size;
  DrawBoxOutline(box.position, half_size, box.rotation, sample_color::kOrange,
                 draw_list);
  DrawOrientation(box, half_size.x(), sample_color::kWhite, draw_list);

  for (std::uint8_t i = 0; i < contact_.point_count; ++i) {
    const ContactPoint& point = contact_.points[i];
    draw_list.circles.push_back(
        {point.position, kPointRadius, sample_color::kYellow});
    // The normal points down into the ground: the depth goes back up to the
    // ground's top
    draw_list.lines.push_back(
        {point.position,
         point.position - contact_.contact_normal * point.penetration,
         sample_color::kRed});
  }
  if (contact_.point_count > 0) {
    const Vec2f from = contact_.points[0].position;
    draw_list.lines.push_back(
        {from, from + contact_.contact_normal, sample_color::kWhite});
  }
}

void ContactManifoldSample::End() {
  world_.Clear();
  contact_ = Contact{};
}

std::array<Vec2f, 2> ContactManifoldSample::incident_corners() const {
  // The same choice as the clipping: the box face most opposed to the
  // ground's normal (up), x faces winning a tie
  const Body& box = world_.body(box_index_);
  const Vec2f half_size =
      world_.oriented_box(world_.collider(box_collider_).shape_index).half_size;
  const Vec2f axis_x = box.rotation.x_axis();
  const Vec2f axis_y = box.rotation.y_axis();
  const bool on_x = Abs(axis_x.y()) >= Abs(axis_y.y());
  const Vec2f axis = on_x ? axis_x : axis_y;
  const Scalar along = on_x ? axis_x.y() : axis_y.y();
  const Vec2f face_normal = along > Scalar{0} ? -axis : axis;
  const Vec2f edge = on_x ? axis_y : axis_x;
  const Scalar normal_half = on_x ? half_size.x() : half_size.y();
  const Scalar edge_half = on_x ? half_size.y() : half_size.x();
  const Vec2f face_center = box.position + face_normal * normal_half;
  return {face_center - edge * edge_half, face_center + edge * edge_half};
}

std::optional<Scalar> ContactManifoldSample::predicted_depth(
    std::size_t corner) const {
  const Scalar depth = ground_top() - incident_corners()[corner].y();
  if (depth < Scalar{0}) {
    return std::nullopt;
  }
  return depth;
}

int ContactManifoldSample::predicted_point_count() const {
  return (predicted_depth(0).has_value() ? 1 : 0) +
         (predicted_depth(1).has_value() ? 1 : 0);
}

std::optional<Scalar> ContactManifoldSample::measured_depth(
    std::size_t corner) const {
  const Vec2f position = incident_corners()[corner];
  for (std::uint8_t i = 0; i < contact_.point_count; ++i) {
    const Vec2f offset = contact_.points[i].position - position;
    if (Abs(offset.x()) < kCornerMatch && Abs(offset.y()) < kCornerMatch) {
      return contact_.points[i].penetration;
    }
  }
  return std::nullopt;
}

Scalar ContactManifoldSample::ground_top() const {
  return world_.body(ground_index_).position.y() + kGroundHalfSize.y();
}

}  // namespace neko::tutorial
