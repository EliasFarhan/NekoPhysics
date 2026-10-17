#include "tutorials/03_contact_manifold/solutions/points_over_a_turn_sample.h"

#include "neko/math/angle.h"

#include "main/samples/body_draw.h"

namespace neko::tutorial {
namespace {
constexpr Vec2f kGroundHalfSize{Scalar{6}, Ratio(1, 2)};
constexpr Vec2f kBoxHalfSize{Scalar{1}, Ratio(1, 2)};
constexpr Scalar kPointRadius = Ratio(1, 16);

// The tilt phi*, in degrees from 0 to 45, up to which both corners of a face
// of half length `length`, with `thickness` behind it, are below the ground:
// the root of c + length sin phi - thickness cos phi, which grows with phi
Scalar Threshold(Scalar c, Scalar length, Scalar thickness) {
  const auto g = [=](Scalar degrees) {
    const Radian phi{Degree{degrees}};
    return c + length * Sin(phi) - thickness * Cos(phi);
  };
  Scalar low{0};
  Scalar high{45};
  if (g(high) <= Scalar{0}) {
    return high;
  }
  if (g(low) > Scalar{0}) {
    return low;
  }
  for (int i = 0; i < 24; ++i) {
    const Scalar middle = (low + high) / Scalar{2};
    if (g(middle) <= Scalar{0}) {
      low = middle;
    } else {
      high = middle;
    }
  }
  return low;
}
}  // namespace

void PointsOverATurnSample::Begin(const SampleContext& context) {
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

  // Center c above the ground's top: the flat bottom face is d deep
  const Scalar c = kBoxHalfSize.y() - parameters_[kSinkDepth].ToScalar();
  box_index_ = world_.AddBody();
  {
    Body& box = world_.body(box_index_);
    box.type = BodyType::kKinematic;
    box.position = {center.x(), ground_top() + c};
    box.angular_velocity =
        Radian{Degree{static_cast<Scalar>(parameters_[kSpin].value)}}.value();
    box_collider_ = world_.AddOrientedBoxCollider(box_index_);
    world_.collider(box_collider_).is_trigger = false;
    world_.oriented_box(world_.collider(box_collider_).shape_index).half_size =
        kBoxHalfSize;
  }

  bottom_threshold_ = Threshold(c, kBoxHalfSize.x(), kBoxHalfSize.y());
  side_threshold_ = Threshold(c, kBoxHalfSize.y(), kBoxHalfSize.x());
  readouts_[0].value = ToDecimal(bottom_threshold_, readouts_[0].scale);
  readouts_[1].value = ToDecimal(side_threshold_, readouts_[1].scale);
  readouts_[2].value =
      ToDecimal(predicted_two_point_steps(), readouts_[2].scale);
  readouts_[6].value = steps_per_turn();
  for (const std::size_t i : {0u, 1u, 2u, 6u}) {
    readouts_[i].is_valid = true;
  }
  UpdateContact();
}

void PointsOverATurnSample::FixedUpdate() {
  world_.Step();
  UpdateContact();
  if (turn_done()) {
    return;
  }
  ++steps_;
  switch (contact_.point_count) {
    case 2:
      ++two_point_steps_;
      break;
    case 1:
      ++one_point_steps_;
      break;
    default:
      ++no_point_steps_;
      break;
  }
  readouts_[3].value = two_point_steps_;
  readouts_[4].value = one_point_steps_;
  readouts_[5].value = no_point_steps_;
  readouts_[3].is_valid = true;
  readouts_[4].is_valid = true;
  readouts_[5].is_valid = true;
}

void PointsOverATurnSample::UpdateContact() {
  contact_ = Contact{};
  if (!world_.DetectContact(
          world_.body(ground_index_), world_.collider(ground_collider_),
          world_.body(box_index_), world_.collider(box_collider_), &contact_)) {
    contact_.point_count = 0;
  }
}

Scalar PointsOverATurnSample::predicted_two_point_steps() const {
  // Each face type: two faces, each from -phi* to +phi*
  const Scalar degrees = Scalar{4} * (bottom_threshold_ + side_threshold_);
  return degrees / Scalar{360} * static_cast<Scalar>(steps_per_turn());
}

int PointsOverATurnSample::steps_per_turn() const {
  // 360 degrees at `spin` degrees per second, 60 steps per second
  return 360 * 60 / parameters_[kSpin].value;
}

void PointsOverATurnSample::Draw(DrawList& draw_list) const {
  const Body& box = world_.body(box_index_);
  draw_list.boxes.push_back({world_.body(ground_index_).position,
                             kGroundHalfSize, sample_color::kDarkGrey});
  DrawBoxOutline(box.position, kBoxHalfSize, box.rotation,
                 sample_color::kOrange, draw_list);
  DrawOrientation(box, kBoxHalfSize.x(), sample_color::kWhite, draw_list);
  const ColorId color =
      contact_.point_count == 2 ? sample_color::kGreen : sample_color::kYellow;
  for (std::uint8_t i = 0; i < contact_.point_count; ++i) {
    draw_list.circles.push_back(
        {contact_.points[i].position, kPointRadius, color});
  }
}

void PointsOverATurnSample::End() {
  world_.Clear();
  contact_ = Contact{};
  steps_ = 0;
  two_point_steps_ = 0;
  one_point_steps_ = 0;
  no_point_steps_ = 0;
  for (auto& readout : readouts_) {
    readout.is_valid = false;
  }
}

Scalar PointsOverATurnSample::ground_top() const {
  return world_.body(ground_index_).position.y() + kGroundHalfSize.y();
}

}  // namespace neko::tutorial
