#include "tutorials/05_friction/solutions/rolling_disk_sample.h"

#include "neko/math/func.h"

#include "main/samples/body_draw.h"

namespace neko::tutorial {
namespace {
constexpr Vec2f kGroundHalfSize{Scalar{6}, Ratio(1, 2)};
constexpr Scalar kGravity = Ratio(981, 100);
// Below this speed the bottom point counts as rolling, in m/s
constexpr Scalar kRollSpeed = Ratio(1, 100);
}  // namespace

void RollingDiskSample::Begin(const SampleContext& context) {
  const Scalar friction = parameters_[kFriction].ToScalar();
  const Scalar radius = parameters_[kRadius].ToScalar();
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

  disk_index_ = world_.AddBody();
  {
    Body& disk = world_.body(disk_index_);
    disk.inverse_mass = Scalar{1};
    disk.position = {left + Scalar{1}, world_.body(ground_index_).position.y() +
                                           kGroundHalfSize.y() + radius};
    disk.velocity = {parameters_[kSpeed].ToScalar(), Scalar{0}};
    start_ = disk.position;
    const ColliderIndex collider_index = world_.AddCircleCollider(disk_index_);
    Collider& collider = world_.collider(collider_index);
    collider.is_trigger = false;
    collider.restitution = Scalar{0};
    collider.friction = friction;
    world_.circle(collider.shape_index).radius = radius;
  }

  readouts_[0].value = ToDecimal(predicted_time(), readouts_[0].scale);
  readouts_[2].value = ToDecimal(predicted_speed(), readouts_[2].scale);
  readouts_[4].value = ToDecimal(predicted_distance(), readouts_[4].scale);
  for (const std::size_t i : {0u, 2u, 4u}) {
    readouts_[i].is_valid = true;
  }
}

void RollingDiskSample::FixedUpdate() {
  world_.Step();
  ++step_;
  if (measured_.has_value()) {
    return;
  }
  const Body& body = disk();
  // The bottom point is at (0, -r) from the center: it moves at v + w x r,
  // whose x part is v.x + w r
  const Scalar slip = body.velocity.x() +
                      body.angular_velocity * parameters_[kRadius].ToScalar();
  if (Abs(slip) < kRollSpeed) {
    measured_ = Rolling{static_cast<Scalar>(step_) * world_.dt(),
                        body.position.x() - start_.x(), body.velocity.x(),
                        body.angular_velocity};
    readouts_[1].value = ToDecimal(measured_->time, readouts_[1].scale);
    readouts_[3].value = ToDecimal(measured_->speed, readouts_[3].scale);
    readouts_[5].value = ToDecimal(measured_->distance, readouts_[5].scale);
    for (const std::size_t i : {1u, 3u, 5u}) {
      readouts_[i].is_valid = true;
    }
  }
}

Scalar RollingDiskSample::predicted_time() const {
  return parameters_[kSpeed].ToScalar() /
         (Scalar{3} * parameters_[kFriction].ToScalar() * kGravity);
}

Scalar RollingDiskSample::predicted_speed() const {
  return parameters_[kSpeed].ToScalar() * Scalar{2} / Scalar{3};
}

Scalar RollingDiskSample::predicted_distance() const {
  const Scalar speed = parameters_[kSpeed].ToScalar();
  return Scalar{5} * speed * speed /
         (Scalar{18} * parameters_[kFriction].ToScalar() * kGravity);
}

void RollingDiskSample::Draw(DrawList& draw_list) const {
  const Body& body = disk();
  const Scalar radius = parameters_[kRadius].ToScalar();
  draw_list.boxes.push_back({world_.body(ground_index_).position,
                             kGroundHalfSize, sample_color::kDarkGrey});
  draw_list.circles.push_back({body.position, radius, sample_color::kOrange});
  const Scalar end_x = start_.x() + predicted_distance();
  draw_list.lines.push_back({{end_x, start_.y() - radius},
                             {end_x, start_.y() + radius * Scalar{2}},
                             sample_color::kRed});
  DrawOrientation(body, radius, sample_color::kWhite, draw_list);
  DrawVelocity(body, sample_color::kSky, draw_list);
}

void RollingDiskSample::End() {
  world_.Clear();
  step_ = 0;
  measured_.reset();
  for (const std::size_t i : {1u, 3u, 5u}) {
    readouts_[i].is_valid = false;
  }
}

}  // namespace neko::tutorial
