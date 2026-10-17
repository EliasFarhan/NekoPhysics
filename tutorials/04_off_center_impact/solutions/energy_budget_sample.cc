#include "tutorials/04_off_center_impact/solutions/energy_budget_sample.h"

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

// The lowest bottom corner of a box turned by `rotation`, from its center
Vec2f LowestCorner(const Rotation& rotation, Vec2f half) {
  const Vec2f left = rotation.Rotate({-half.x(), -half.y()});
  const Vec2f right = rotation.Rotate({half.x(), -half.y()});
  return left.y() < right.y() ? left : right;
}
}  // namespace

void EnergyBudgetSample::Begin(const SampleContext& context) {
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
    box.rotation.set_angle(
        Radian{Degree{static_cast<Scalar>(parameters_[kTilt].value)}});
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
  readouts_[0].value = ToDecimal(energy_before(), readouts_[0].scale);
  readouts_[6].value = ToDecimal(flat_loss(), readouts_[6].scale);
  readouts_[0].is_valid = true;
  readouts_[6].is_valid = true;
}

void EnergyBudgetSample::FixedUpdate() {
  const Rotation rotation_before = world_.body(box_index_).rotation;
  world_.Step();
  if (lever_arm_.has_value() || world_.touching_pairs().empty()) {
    return;
  }
  const Body& box = world_.body(box_index_);
  lever_arm_ = LowestCorner(rotation_before, half_size()).x();
  velocity_after_ = box.velocity.y();
  spin_after_ = box.angular_velocity;

  readouts_[1].value = ToDecimal(*linear_energy_after(), readouts_[1].scale);
  readouts_[2].value = ToDecimal(*rotation_energy_after(), readouts_[2].scale);
  readouts_[3].value = ToDecimal(
      *linear_energy_after() + *rotation_energy_after(), readouts_[3].scale);
  readouts_[4].value = ToDecimal(*predicted_loss(), readouts_[4].scale);
  readouts_[5].value = ToDecimal(*measured_loss(), readouts_[5].scale);
  for (auto& readout : readouts_) {
    readout.is_valid = true;
  }
}

Scalar EnergyBudgetSample::inertia() const {
  const Scalar width = parameters_[kWidth].ToScalar();
  const Scalar height = kBoxHalfHeight * Scalar{2};
  return (width * width + height * height) / Scalar{12};  // m = 1 kg
}

Scalar EnergyBudgetSample::energy_before() const {
  return kSpeed * kSpeed / Scalar{2};  // m = 1 kg
}

std::optional<Scalar> EnergyBudgetSample::linear_energy_after() const {
  if (!velocity_after_.has_value()) {
    return std::nullopt;
  }
  // The impulse is vertical: the box keeps no horizontal velocity
  return *velocity_after_ * *velocity_after_ / Scalar{2};
}

std::optional<Scalar> EnergyBudgetSample::rotation_energy_after() const {
  if (!spin_after_.has_value()) {
    return std::nullopt;
  }
  return inertia() * *spin_after_ * *spin_after_ / Scalar{2};
}

std::optional<Scalar> EnergyBudgetSample::predicted_loss() const {
  if (!lever_arm_.has_value()) {
    return std::nullopt;
  }
  const Scalar e = parameters_[kRestitution].ToScalar();
  const Scalar k = Scalar{1} + *lever_arm_ * *lever_arm_ / inertia();
  return (Scalar{1} - e * e) * kSpeed * kSpeed / (Scalar{2} * k);
}

std::optional<Scalar> EnergyBudgetSample::measured_loss() const {
  if (!lever_arm_.has_value()) {
    return std::nullopt;
  }
  return energy_before() - *linear_energy_after() - *rotation_energy_after();
}

Scalar EnergyBudgetSample::flat_loss() const {
  const Scalar e = parameters_[kRestitution].ToScalar();
  return (Scalar{1} - e * e) * kSpeed * kSpeed / Scalar{2};
}

Vec2f EnergyBudgetSample::half_size() const {
  return {parameters_[kWidth].ToScalar() / Scalar{2}, kBoxHalfHeight};
}

void EnergyBudgetSample::Draw(DrawList& draw_list) const {
  const Body& box = world_.body(box_index_);
  draw_list.boxes.push_back({world_.body(ground_index_).position,
                             kGroundHalfSize, sample_color::kDarkGrey});
  DrawBoxOutline(box.position, half_size(), box.rotation, sample_color::kOrange,
                 draw_list);
  DrawOrientation(box, half_size().x(), sample_color::kWhite, draw_list);
  DrawVelocity(box, sample_color::kSky, draw_list);
}

void EnergyBudgetSample::End() {
  world_.Clear();
  lever_arm_.reset();
  velocity_after_.reset();
  spin_after_.reset();
  for (auto& readout : readouts_) {
    readout.is_valid = false;
  }
}

}  // namespace neko::tutorial
