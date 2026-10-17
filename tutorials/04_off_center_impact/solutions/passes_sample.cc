#include "tutorials/04_off_center_impact/solutions/passes_sample.h"

#include "neko/math/func.h"

#include "main/samples/body_draw.h"

namespace neko::tutorial {
namespace {
constexpr Vec2f kGroundHalfSize{Scalar{6}, Ratio(1, 2)};
constexpr Vec2f kBoxHalfSize{Scalar{1}, Ratio(1, 2)};
// Height of the box's bottom above the ground at the start, in meters
constexpr Scalar kStartHeight = Ratio(3, 2);
constexpr Scalar kSpeed{2};
}  // namespace

void PassesSample::Begin(const SampleContext& context) {
  world_ = PhysicsWorld{{.gravity = Vec2f::Zero(),
                         .dt = kFixedDeltaTime,
                         .max_bodies = 2,
                         .max_colliders = 2,
                         .max_touching_pairs = 1,
                         .max_broadphase_pairs = 1,
                         .velocity_iterations = parameters_[kPasses].value}};
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
    const Scalar ground_top =
        world_.body(ground_index_).position.y() + kGroundHalfSize.y();
    box.position = {center.x(), ground_top + kStartHeight + kBoxHalfSize.y()};
    box.velocity = {Scalar{0}, -kSpeed};
    const ColliderIndex collider_index =
        world_.AddOrientedBoxCollider(box_index_);
    Collider& collider = world_.collider(collider_index);
    collider.is_trigger = false;
    collider.restitution = parameters_[kRestitution].ToScalar();
    world_.oriented_box(collider.shape_index).half_size = kBoxHalfSize;
  }

  const Motion motion = predicted(parameters_[kPasses].value);
  readouts_[0].value = ToDecimal(motion.velocity, readouts_[0].scale);
  readouts_[2].value = ToDecimal(motion.spin, readouts_[2].scale);
  readouts_[4].value = ToDecimal(exact_velocity(), readouts_[4].scale);
  readouts_[0].is_valid = true;
  readouts_[2].is_valid = true;
  readouts_[4].is_valid = true;
}

void PassesSample::FixedUpdate() {
  const Vec2f position_before = world_.body(box_index_).position;
  world_.Step();
  if (measured_.has_value() || world_.touching_pairs().empty()) {
    return;
  }
  const Body& box = world_.body(box_index_);
  measured_ = Motion{box.velocity.y(), box.angular_velocity};
  impact_center_ = position_before;
  readouts_[1].value = ToDecimal(measured_->velocity, readouts_[1].scale);
  readouts_[3].value = ToDecimal(measured_->spin, readouts_[3].scale);
  readouts_[1].is_valid = true;
  readouts_[3].is_valid = true;
}

PassesSample::Motion PassesSample::predicted(int passes) const {
  // The solver's loop by hand, from the box's side: the normal pushes it up,
  // a point at x from the center has the lever arm x
  const Scalar inverse_mass = Scalar{1};
  const Scalar inverse_inertia = world_.ComputeInverseInertia(box_index_);
  const Scalar target = exact_velocity();  // e times the approach speed
  const std::array<Scalar, 2> lever = {-kBoxHalfSize.x(), kBoxHalfSize.x()};
  std::array<Scalar, 2> total{};
  Motion motion{-kSpeed, Scalar{0}};
  for (int pass = 0; pass < passes; ++pass) {
    for (std::size_t i = 0; i < 2; ++i) {
      const Scalar normal_mass =
          Scalar{1} / (inverse_mass + lever[i] * lever[i] * inverse_inertia);
      const Scalar approach = motion.velocity + motion.spin * lever[i];
      const Scalar new_total =
          Max(total[i] + (target - approach) * normal_mass, Scalar{0});
      const Scalar impulse = new_total - total[i];
      total[i] = new_total;
      motion.velocity += impulse * inverse_mass;
      motion.spin += lever[i] * impulse * inverse_inertia;
    }
  }
  return motion;
}

Scalar PassesSample::exact_velocity() const {
  return parameters_[kRestitution].ToScalar() * kSpeed;
}

void PassesSample::Draw(DrawList& draw_list) const {
  const Body& box = world_.body(box_index_);
  draw_list.boxes.push_back({world_.body(ground_index_).position,
                             kGroundHalfSize, sample_color::kDarkGrey});
  DrawBoxOutline(box.position, kBoxHalfSize, box.rotation,
                 sample_color::kOrange, draw_list);
  DrawOrientation(box, kBoxHalfSize.x(), sample_color::kWhite, draw_list);
  DrawVelocity(box, sample_color::kSky, draw_list);
  if (measured_.has_value()) {
    for (const Scalar side : {-kBoxHalfSize.x(), kBoxHalfSize.x()}) {
      draw_list.circles.push_back(
          {impact_center_ + Vec2f{side, -kBoxHalfSize.y()}, Ratio(1, 16),
           sample_color::kYellow});
    }
  }
}

void PassesSample::End() {
  world_.Clear();
  measured_.reset();
  for (auto& readout : readouts_) {
    readout.is_valid = false;
  }
}

}  // namespace neko::tutorial
