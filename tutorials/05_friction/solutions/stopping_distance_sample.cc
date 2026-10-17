#include "tutorials/05_friction/solutions/stopping_distance_sample.h"

#include "main/samples/body_draw.h"

namespace neko::tutorial {
namespace {
constexpr Vec2f kGroundHalfSize{Scalar{6}, Ratio(1, 2)};
constexpr Vec2f kBoxHalfSize{Ratio(1, 2), Ratio(1, 2)};
constexpr Scalar kGravity = Ratio(981, 100);
// Below this speed the box counts as stopped, in m/s
constexpr Scalar kStopSpeed = Ratio(1, 1000);
}  // namespace

void StoppingDistanceSample::Begin(const SampleContext& context) {
  const Scalar friction = parameters_[kFriction].ToScalar();
  const Vec2f center = context.world_bounds.GetCenter();
  const Scalar left = context.world_bounds.min_bound().x();

  ground_index_ = world_.AddBody();
  {
    Body& ground = world_.body(ground_index_);
    ground.type = BodyType::kStatic;
    ground.inverse_mass = Scalar{0};
    ground.position = center - Vec2f{Scalar{0}, Scalar{2}};
    const ColliderIndex collider_index = world_.AddAabbCollider(ground_index_);
    Collider& collider = world_.collider(collider_index);
    collider.is_trigger = false;
    collider.restitution = Scalar{0};
    collider.friction = friction;
    world_.aabb(collider.shape_index).half_size = kGroundHalfSize;
  }

  box_index_ = world_.AddBody();
  {
    Body& box = world_.body(box_index_);
    box.inverse_mass = Scalar{1};
    // Resting on the ground near its left end, thrown to the right
    box.position = {left + Scalar{1}, world_.body(ground_index_).position.y() +
                                          kGroundHalfSize.y() +
                                          kBoxHalfSize.y()};
    box.velocity = {parameters_[kSpeed].ToScalar(), Scalar{0}};
    start_ = box.position;
    const ColliderIndex collider_index =
        world_.AddOrientedBoxCollider(box_index_);
    Collider& collider = world_.collider(collider_index);
    collider.is_trigger = false;
    collider.restitution = Scalar{0};
    collider.friction = friction;
    world_.oriented_box(collider.shape_index).half_size = kBoxHalfSize;
  }

  readouts_[0].value = ToDecimal(predicted_time(), readouts_[0].scale);
  readouts_[2].value = ToDecimal(predicted_distance(), readouts_[2].scale);
  readouts_[0].is_valid = true;
  readouts_[2].is_valid = true;
}

void StoppingDistanceSample::FixedUpdate() {
  world_.Step();
  ++step_;
  if (measured_time_.has_value()) {
    return;
  }
  const Body& box = world_.body(box_index_);
  if (box.velocity.x() < kStopSpeed) {
    measured_time_ = static_cast<Scalar>(step_) * world_.dt();
    measured_distance_ = box.position.x() - start_.x();
    readouts_[1].value = ToDecimal(*measured_time_, readouts_[1].scale);
    readouts_[3].value = ToDecimal(*measured_distance_, readouts_[3].scale);
    readouts_[1].is_valid = true;
    readouts_[3].is_valid = true;
  }
}

Scalar StoppingDistanceSample::predicted_time() const {
  return parameters_[kSpeed].ToScalar() /
         (parameters_[kFriction].ToScalar() * kGravity);
}

Scalar StoppingDistanceSample::predicted_distance() const {
  const Scalar speed = parameters_[kSpeed].ToScalar();
  return speed * speed /
         (Scalar{2} * parameters_[kFriction].ToScalar() * kGravity);
}

void StoppingDistanceSample::Draw(DrawList& draw_list) const {
  const Body& box = world_.body(box_index_);
  draw_list.boxes.push_back({world_.body(ground_index_).position,
                             kGroundHalfSize, sample_color::kDarkGrey});
  const Scalar stop_x = start_.x() + predicted_distance();
  draw_list.lines.push_back(
      {{stop_x, start_.y() - kBoxHalfSize.y()},
       {stop_x, start_.y() + kBoxHalfSize.y() * Scalar{3}},
       sample_color::kRed});
  DrawBoxOutline(box.position, kBoxHalfSize, box.rotation,
                 sample_color::kOrange, draw_list);
  DrawOrientation(box, kBoxHalfSize.x(), sample_color::kWhite, draw_list);
  DrawVelocity(box, sample_color::kSky, draw_list);
}

void StoppingDistanceSample::End() {
  world_.Clear();
  step_ = 0;
  measured_time_.reset();
  measured_distance_.reset();
  readouts_[1].is_valid = false;
  readouts_[3].is_valid = false;
}

}  // namespace neko::tutorial
