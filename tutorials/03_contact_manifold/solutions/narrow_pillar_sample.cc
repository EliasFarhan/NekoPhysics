#include "tutorials/03_contact_manifold/solutions/narrow_pillar_sample.h"

#include "neko/math/angle.h"
#include "neko/math/func.h"

#include "main/samples/body_draw.h"

namespace neko::tutorial {
namespace {
constexpr Vec2f kPillarHalfSize{Ratio(1, 2), Scalar{1}};
constexpr Vec2f kBoxHalfSize{Ratio(3, 2), Ratio(1, 2)};
constexpr Scalar kPointRadius = Ratio(1, 16);
}  // namespace

void NarrowPillarSample::Begin(const SampleContext& context) {
  const Vec2f center = context.world_bounds.GetCenter();

  pillar_index_ = world_.AddBody();
  {
    Body& pillar = world_.body(pillar_index_);
    pillar.type = BodyType::kStatic;
    pillar.inverse_mass = Scalar{0};
    pillar.position = center - Vec2f{Scalar{1}, Ratio(3, 2)};
    pillar_collider_ = world_.AddAabbCollider(pillar_index_);
    world_.collider(pillar_collider_).is_trigger = false;
    world_.aabb(world_.collider(pillar_collider_).shape_index).half_size =
        kPillarHalfSize;
  }

  box_index_ = world_.AddBody();
  {
    Body& box = world_.body(box_index_);
    box.type = BodyType::kKinematic;
    box.position = {
        pillar_center().x() + parameters_[kOffset].ToScalar(),
        pillar_top() + kBoxHalfSize.y() - parameters_[kSinkDepth].ToScalar()};
    box.rotation.set_angle(
        Radian{Degree{static_cast<Scalar>(parameters_[kAngle].value)}});
    box_collider_ = world_.AddOrientedBoxCollider(box_index_);
    world_.collider(box_collider_).is_trigger = false;
    world_.oriented_box(world_.collider(box_collider_).shape_index).half_size =
        kBoxHalfSize;
  }
  UpdateContact();
}

void NarrowPillarSample::FixedUpdate() {
  world_.Step();
  UpdateContact();
}

void NarrowPillarSample::UpdateContact() {
  // The narrowphase's own call, pillar first (lower collider index)
  contact_ = Contact{};
  if (!world_.DetectContact(
          world_.body(pillar_index_), world_.collider(pillar_collider_),
          world_.body(box_index_), world_.collider(box_collider_), &contact_)) {
    contact_.point_count = 0;
  }

  const bool reference = pillar_is_reference();
  readouts_[0].value = reference ? 1 : 2;
  readouts_[0].is_valid = contact_.point_count > 0;
  const Prediction predicted = prediction();
  readouts_[1].value = predicted.count;
  readouts_[1].is_valid = reference;
  readouts_[2].value = contact_.point_count;
  readouts_[2].is_valid = true;
  const Scalar pillar_x = pillar_center().x();
  for (std::size_t i = 0; i < 2; ++i) {
    SampleReadout* const row = &readouts_[3 + 4 * i];
    const bool has_predicted =
        reference && static_cast<int>(i) < predicted.count;
    const bool has_measured = i < contact_.point_count;
    row[0].is_valid = has_predicted;
    row[2].is_valid = has_predicted;
    row[1].is_valid = has_measured;
    row[3].is_valid = has_measured;
    if (has_predicted) {
      row[0].value =
          ToDecimal(predicted.points[i].position.x() - pillar_x, row[0].scale);
      row[2].value = ToDecimal(predicted.points[i].depth, row[2].scale);
    }
    if (has_measured) {
      row[1].value =
          ToDecimal(contact_.points[i].position.x() - pillar_x, row[1].scale);
      row[3].value = ToDecimal(contact_.points[i].penetration, row[3].scale);
    }
  }
}

NarrowPillarSample::Prediction NarrowPillarSample::prediction() const {
  // The incident face: the box face most opposed to the pillar's top normal
  // (up). For |a| < 45 degrees it is the bottom face, from the left corner to
  // the right one.
  const Body& box = world_.body(box_index_);
  Vec2f v1 = box.position + box.rotation.Rotate(-kBoxHalfSize);
  Vec2f v2 =
      box.position + box.rotation.Rotate({kBoxHalfSize.x(), -kBoxHalfSize.y()});
  // The side planes of the pillar's top face, x = left and x = right: an end
  // outside a plane slides along the edge onto it
  const Scalar left = pillar_center().x() - kPillarHalfSize.x();
  const Scalar right = pillar_center().x() + kPillarHalfSize.x();
  const auto clip = [](Vec2f& outside, Vec2f inside, Scalar plane_x) {
    const Scalar t = (plane_x - outside.x()) / (inside.x() - outside.x());
    outside = outside + (inside - outside) * t;
  };
  Prediction result;
  if (v2.x() < left || v1.x() > right) {
    return result;
  }
  if (v1.x() < left) {
    clip(v1, v2, left);
  }
  if (v2.x() > right) {
    clip(v2, v1, right);
  }
  // The clipped ends below the top are the manifold
  for (const Vec2f end : {v1, v2}) {
    const Scalar depth = pillar_top() - end.y();
    if (depth >= Scalar{0}) {
      result.points[static_cast<std::size_t>(result.count)] = {end, depth};
      ++result.count;
    }
  }
  return result;
}

bool NarrowPillarSample::pillar_is_reference() const {
  return contact_.point_count > 0 && contact_.contact_normal.x() == Scalar{0} &&
         contact_.contact_normal.y() < Scalar{0};
}

void NarrowPillarSample::Draw(DrawList& draw_list) const {
  const Body& box = world_.body(box_index_);
  draw_list.boxes.push_back(
      {pillar_center(), kPillarHalfSize, sample_color::kDarkGrey});
  DrawBoxOutline(box.position, kBoxHalfSize, box.rotation,
                 sample_color::kOrange, draw_list);
  if (pillar_is_reference()) {
    const Prediction predicted = prediction();
    for (int i = 0; i < predicted.count; ++i) {
      draw_list.circles.push_back(
          {predicted.points[static_cast<std::size_t>(i)].position,
           kPointRadius * Scalar{2}, sample_color::kGreen});
    }
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

void NarrowPillarSample::End() {
  world_.Clear();
  contact_ = Contact{};
}

Vec2f NarrowPillarSample::pillar_center() const {
  return world_.body(pillar_index_).position;
}

Scalar NarrowPillarSample::pillar_top() const {
  return pillar_center().y() + kPillarHalfSize.y();
}

}  // namespace neko::tutorial
