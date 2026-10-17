#include "tutorials/04_off_center_impact/solutions/fastest_spin_sample.h"

#include "neko/math/angle.h"
#include "neko/math/func.h"
#include "neko/math/rotation.h"

#include "main/samples/body_draw.h"

namespace neko::tutorial {
namespace {
constexpr Vec2f kGroundHalfSize{Scalar{6}, Ratio(1, 2)};
constexpr Scalar kBoxHalfHeight = Ratio(1, 2);
// Height of the box's lowest corner above the ground at the start, in meters
constexpr Scalar kStartHeight = Ratio(3, 2);
constexpr Scalar kSpeed{2};
constexpr int kFirstTilt = 5;
constexpr int kLastTilt = 40;

// The lowest bottom corner of a box turned by `rotation`, from its center
Vec2f LowestCorner(const Rotation& rotation, Vec2f half) {
  const Vec2f left = rotation.Rotate({-half.x(), -half.y()});
  const Vec2f right = rotation.Rotate({half.x(), -half.y()});
  return left.y() < right.y() ? left : right;
}

Rotation TiltRotation(int degrees) {
  Rotation rotation;
  rotation.set_angle(Radian{Degree{static_cast<Scalar>(degrees)}});
  return rotation;
}
}  // namespace

void FastestSpinSample::Begin(const SampleContext& context) {
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
    collider.restitution = Scalar{0};
    collider.friction = Scalar{0};
    world_.aabb(collider.shape_index).half_size = kGroundHalfSize;
  }

  box_index_ = world_.AddBody();
  {
    Body& box = world_.body(box_index_);
    box.inverse_mass = Scalar{1};
    box.rotation = TiltRotation(parameters_[kTilt].value);
    const Scalar ground_top =
        world_.body(ground_index_).position.y() + kGroundHalfSize.y();
    box.position = {center.x(),
                    ground_top + kStartHeight -
                        LowestCorner(box.rotation, half_size()).y()};
    box.velocity = {Scalar{0}, -kSpeed};
    const ColliderIndex collider_index =
        world_.AddOrientedBoxCollider(box_index_);
    Collider& collider = world_.collider(collider_index);
    collider.is_trigger = false;
    collider.restitution = parameters_[kRestitution].ToScalar();
    world_.oriented_box(collider.shape_index).half_size = half_size();
  }

  const int tilt = parameters_[kTilt].value;
  readouts_[0].value = ToDecimal(lever_arm(tilt), readouts_[0].scale);
  readouts_[1].value = ToDecimal(gyration_radius(), readouts_[1].scale);
  readouts_[2].value = ToDecimal(predicted_spin(tilt), readouts_[2].scale);
  readouts_[4].value = fastest_tilt();
  readouts_[5].value = ToDecimal(spin_bound(), readouts_[5].scale);
  for (const std::size_t i : {0u, 1u, 2u, 4u, 5u}) {
    readouts_[i].is_valid = true;
  }
}

void FastestSpinSample::FixedUpdate() {
  // The contact is found and solved with the box where it was at the start
  // of the step
  const Body& before = world_.body(box_index_);
  const Vec2f position_before = before.position;
  const Rotation rotation_before = before.rotation;
  world_.Step();
  if (measured_spin_.has_value() || world_.touching_pairs().empty()) {
    return;
  }
  measured_spin_ = world_.body(box_index_).angular_velocity;
  impact_center_ = position_before;
  impact_point_ = position_before + LowestCorner(rotation_before, half_size());
  readouts_[3].value = ToDecimal(*measured_spin_, readouts_[3].scale);
  readouts_[3].is_valid = true;
}

Scalar FastestSpinSample::inertia() const {
  const Scalar width = parameters_[kWidth].ToScalar();
  const Scalar height = kBoxHalfHeight * Scalar{2};
  return (width * width + height * height) / Scalar{12};  // m = 1 kg
}

Scalar FastestSpinSample::gyration_radius() const {
  return Sqrt(inertia());  // sqrt(I / m), m = 1 kg
}

Scalar FastestSpinSample::lever_arm(int tilt_degrees) const {
  return Abs(LowestCorner(TiltRotation(tilt_degrees), half_size()).x());
}

Scalar FastestSpinSample::predicted_spin(int tilt_degrees) const {
  // The lever arm r x n of the corner, n = (0, 1): its x
  const Scalar lever =
      LowestCorner(TiltRotation(tilt_degrees), half_size()).x();
  const Scalar e = parameters_[kRestitution].ToScalar();
  const Scalar impulse =
      (Scalar{1} + e) * kSpeed / (Scalar{1} + lever * lever / inertia());
  return lever * impulse / inertia();
}

int FastestSpinSample::fastest_tilt() const {
  int best = kFirstTilt;
  for (int tilt = kFirstTilt + 1; tilt <= kLastTilt; ++tilt) {
    if (Abs(predicted_spin(tilt)) > Abs(predicted_spin(best))) {
      best = tilt;
    }
  }
  return best;
}

Scalar FastestSpinSample::spin_bound() const {
  const Scalar e = parameters_[kRestitution].ToScalar();
  return (Scalar{1} + e) * kSpeed / (Scalar{2} * gyration_radius());
}

Vec2f FastestSpinSample::half_size() const {
  return {parameters_[kWidth].ToScalar() / Scalar{2}, kBoxHalfHeight};
}

void FastestSpinSample::Draw(DrawList& draw_list) const {
  const Body& box = world_.body(box_index_);
  draw_list.boxes.push_back({world_.body(ground_index_).position,
                             kGroundHalfSize, sample_color::kDarkGrey});
  DrawBoxOutline(box.position, half_size(), box.rotation, sample_color::kOrange,
                 draw_list);
  DrawOrientation(box, half_size().x(), sample_color::kWhite, draw_list);
  DrawVelocity(box, sample_color::kSky, draw_list);
  if (measured_spin_.has_value()) {
    draw_list.lines.push_back(
        {impact_center_, impact_point_, sample_color::kGrey});
    draw_list.circles.push_back(
        {impact_point_, Ratio(1, 16), sample_color::kYellow});
  }
}

void FastestSpinSample::End() {
  world_.Clear();
  measured_spin_.reset();
  for (auto& readout : readouts_) {
    readout.is_valid = false;
  }
}

}  // namespace neko::tutorial
