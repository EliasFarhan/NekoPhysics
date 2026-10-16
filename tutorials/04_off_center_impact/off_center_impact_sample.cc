#include "tutorials/04_off_center_impact/off_center_impact_sample.h"

#include "neko/math/angle.h"

#include "main/samples/body_draw.h"

namespace neko::tutorial {
namespace {
constexpr Vec2f kGroundHalfSize{Scalar{6}, Ratio(1, 2)};
constexpr Scalar kBoxHalfHeight = Ratio(1, 2);
// Height of the box's lowest corner above the ground at the start, in meters
constexpr Scalar kStartHeight = Ratio(3, 2);
// Impulse drawn as an arrow of this many meters per N s
constexpr Scalar kImpulseDrawScale = Ratio(1, 2);
}  // namespace

void OffCenterImpactSample::Begin(const SampleContext& context) {
  const Vec2f center = context.world_bounds.GetCenter();

  ground_index_ = world_.AddBody();
  {
    Body& ground = world_.body(ground_index_);
    ground.type = BodyType::kStatic;
    ground.inverse_mass = Scalar{0};
    ground.position = center - Vec2f{Scalar{0}, Ratio(5, 2)};
    const ColliderIndex collider_index = world_.AddAabbCollider(ground_index_);
    Collider& collider = world_.collider(collider_index);
    collider.is_trigger = false;
    // A contact uses the larger restitution: 0 lets the box's decide
    collider.restitution = Scalar{0};
    world_.aabb(collider.shape_index).half_size = kGroundHalfSize;
  }

  box_index_ = world_.AddBody();
  {
    Body& box = world_.body(box_index_);
    box.inverse_mass = Scalar{1};
    box.rotation.set_angle(
        Radian{Degree{static_cast<Scalar>(parameters_[kTilt].value)}});
    // Lowest bottom corner kStartHeight above the ground
    const Vec2f half = half_size();
    const Scalar left = box.rotation.Rotate({-half.x(), -half.y()}).y();
    const Scalar right = box.rotation.Rotate({half.x(), -half.y()}).y();
    const Scalar ground_top =
        world_.body(ground_index_).position.y() + kGroundHalfSize.y();
    box.position = {center.x(), ground_top + kStartHeight - Min(left, right)};
    box.velocity = {Scalar{0}, -parameters_[kSpeed].ToScalar()};
    const ColliderIndex collider_index =
        world_.AddOrientedBoxCollider(box_index_);
    Collider& collider = world_.collider(collider_index);
    collider.is_trigger = false;
    collider.restitution = restitution();
    world_.oriented_box(collider.shape_index).half_size = half;
  }
}

void OffCenterImpactSample::FixedUpdate() {
  // The contact is found and solved with the box where it was at the start
  // of the step: the step moves and turns it afterwards
  const Body& before = world_.body(box_index_);
  const Vec2f velocity_before = before.velocity;
  const Vec2f position_before = before.position;
  const Rotation rotation_before = before.rotation;
  world_.Step();
  if (lever_arm_.has_value() || world_.touching_pairs().empty()) {
    return;
  }
  const Body& box = world_.body(box_index_);
  const Vec2f half = half_size();
  const Vec2f left = rotation_before.Rotate({-half.x(), -half.y()});
  const Vec2f right = rotation_before.Rotate({half.x(), -half.y()});
  const Vec2f r = left.y() < right.y() ? left : right;
  lever_arm_ = Vec2f::Cross(r, Vec2f::Up());
  approach_velocity_ = velocity_before.y();
  measured_velocity_ = box.velocity.y();
  measured_spin_ = box.angular_velocity;
  impact_center_ = position_before;
  impact_point_ = position_before + r;

  readouts_[0].value = ToDecimal(*lever_arm_, readouts_[0].scale);
  readouts_[1].value = ToDecimal(*predicted_spin(), readouts_[1].scale);
  readouts_[2].value = ToDecimal(*measured_spin_, readouts_[2].scale);
  readouts_[3].value = ToDecimal(*predicted_velocity(), readouts_[3].scale);
  readouts_[4].value = ToDecimal(*measured_velocity_, readouts_[4].scale);
  readouts_[5].value = ToDecimal(*corner_rebound(), readouts_[5].scale);
  for (auto& readout : readouts_) {
    readout.is_valid = true;
  }
}

void OffCenterImpactSample::Draw(DrawList& draw_list) const {
  const Body& ground = world_.body(ground_index_);
  const Body& box = world_.body(box_index_);
  draw_list.boxes.push_back(
      {ground.position, kGroundHalfSize, sample_color::kDarkGrey});
  const Vec2f half = half_size();
  DrawBoxOutline(box.position, half, box.rotation, sample_color::kOrange,
                 draw_list);
  DrawOrientation(box, half.x(), sample_color::kWhite, draw_list);
  DrawVelocity(box, sample_color::kSky, draw_list);
  if (lever_arm_.has_value()) {
    // Where the hit was: the lever arm from the center at the impact to the
    // corner, and the impulse
    draw_list.lines.push_back(
        {impact_center_, impact_point_, sample_color::kGrey});
    draw_list.circles.push_back(
        {impact_point_, Ratio(1, 16), sample_color::kYellow});
    draw_list.lines.push_back(
        {impact_point_,
         impact_point_ +
             Vec2f::Up() * (*predicted_impulse() * kImpulseDrawScale),
         sample_color::kRed});
  }
}

void OffCenterImpactSample::End() {
  world_.Clear();
  lever_arm_.reset();
  approach_velocity_.reset();
  measured_velocity_.reset();
  measured_spin_.reset();
  for (auto& readout : readouts_) {
    readout.is_valid = false;
  }
}

Scalar OffCenterImpactSample::inertia() const {
  const Scalar width = parameters_[kWidth].ToScalar();
  const Scalar height = kBoxHalfHeight * Scalar{2};
  return (width * width + height * height) / Scalar{12};  // m = 1 kg
}

std::optional<Scalar> OffCenterImpactSample::predicted_impulse() const {
  if (!lever_arm_.has_value()) {
    return std::nullopt;
  }
  const Scalar lever = *lever_arm_;
  const Scalar effective_inverse_mass = Scalar{1} + lever * lever / inertia();
  return -(Scalar{1} + restitution()) * *approach_velocity_ /
         effective_inverse_mass;
}

std::optional<Scalar> OffCenterImpactSample::predicted_velocity() const {
  const auto impulse = predicted_impulse();
  if (!impulse.has_value()) {
    return std::nullopt;
  }
  return *approach_velocity_ + *impulse;  // m = 1 kg
}

std::optional<Scalar> OffCenterImpactSample::predicted_spin() const {
  const auto impulse = predicted_impulse();
  if (!impulse.has_value()) {
    return std::nullopt;
  }
  return *lever_arm_ * *impulse / inertia();
}

std::optional<Scalar> OffCenterImpactSample::corner_rebound() const {
  if (!lever_arm_.has_value() || *approach_velocity_ == Scalar{0}) {
    return std::nullopt;
  }
  // The corner moves at v + w x r: its vertical part is v.y + w r.x
  const Scalar corner_after =
      *measured_velocity_ + *measured_spin_ * *lever_arm_;
  return -corner_after / *approach_velocity_;
}

Vec2f OffCenterImpactSample::half_size() const {
  return {parameters_[kWidth].ToScalar() / Scalar{2}, kBoxHalfHeight};
}

}  // namespace neko::tutorial
