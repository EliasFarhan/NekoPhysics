#include "tutorials/14_moving_platform/solutions/teleport_warm_start_sample.h"

#include <algorithm>

#include "neko/math/func.h"

#include "main/samples/body_draw.h"

namespace neko::tutorial {
namespace {
constexpr Scalar kGroundTop = Ratio(1, 2);
constexpr Vec2f kBoxHalfSize{Ratio(1, 4), Ratio(1, 4)};
constexpr Scalar kStackX = Ratio(16, 5);
constexpr Scalar kDestinationX = Ratio(48, 5);
// Each box of the stack this much to the right of the one below
constexpr Scalar kStackShift = Ratio(3, 100);
}  // namespace

void TeleportWarmStartSample::Begin(const SampleContext& context) {
  const Scalar left = context.world_bounds.min_bound().x();
  const Vec2f bounds_center = context.world_bounds.GetCenter();
  const Scalar half_width = context.world_bounds.GetHalfSize().x();
  world_ =
      PhysicsWorld{{.gravity = Vec2f{Scalar{0}, -Ratio(981, 100)},
                    .dt = kFixedDeltaTime,
                    .max_bodies = kMaxBoxes + 1,
                    .max_colliders = kMaxBoxes + 1,
                    .max_touching_pairs = 16,
                    .max_broadphase_pairs = 32,
                    .max_joints = 0,
                    .max_passing_pairs = 0,
                    .velocity_iterations = parameters_[kIterations].value}};
  ground_ = Aabbf::FromCenter({bounds_center.x(), kGroundTop / Scalar{2}},
                              {half_width, kGroundTop / Scalar{2}});
  {
    const BodyIndex ground = world_.AddBody();
    world_.body(ground).type = BodyType::kStatic;
    world_.body(ground).inverse_mass = Scalar{0};
    world_.body(ground).position = ground_.GetCenter();
    const ColliderIndex collider_index = world_.AddAabbCollider(ground);
    Collider& collider = world_.collider(collider_index);
    collider.is_trigger = false;
    collider.restitution = Scalar{0};
    world_.aabb(collider.shape_index).half_size = ground_.GetHalfSize();
  }
  box_count_ = static_cast<std::size_t>(parameters_[kStack].value) + 1;
  for (std::size_t i = 0; i < box_count_; ++i) {
    const auto level = static_cast<Scalar>(static_cast<int>(i));
    const BodyIndex body = world_.AddBody();
    world_.body(body).position = {
        left + kStackX + kStackShift * level,
        kGroundTop + kBoxHalfSize.y() + Scalar{2} * kBoxHalfSize.y() * level};
    boxes_[i] = world_.AddOrientedBoxCollider(body);
    Collider& collider = world_.collider(boxes_[i]);
    collider.is_trigger = false;
    collider.restitution = Scalar{0};
    world_.oriented_box(collider.shape_index).half_size = kBoxHalfSize;
  }
  destination_x_ = left + kDestinationX;
}

void TeleportWarmStartSample::FixedUpdate() {
  const BodyIndex bottom = world_.collider(boxes_[0]).body_index;
  if (step_ == kTeleportStep) {
    Body& body = world_.body(bottom);
    moved_y_ = body.position.y();
    const Vec2f there{destination_x_, moved_y_};
    if (parameters_[kMode].value == 0) {
      world_.Teleport(bottom, there, body.rotation.angle());
    } else {
      // By hand: the world still remembers the contacts of the old place
      body.position = there;
    }
  }
  world_.Step();
  ++step_;
  if (step_ > kTeleportStep && step_ <= kMeasuredSteps) {
    const Body& body = world_.body(bottom);
    max_spin_ = std::max(max_spin_, Abs(body.angular_velocity));
    max_rise_ = std::max(max_rise_, body.position.y() - moved_y_);
    readouts_[0].value = ToDecimal(max_spin_, readouts_[0].scale);
    readouts_[1].value =
        ToDecimal(max_rise_ * Scalar{1000}, readouts_[1].scale);
    readouts_[0].is_valid = true;
    readouts_[1].is_valid = true;
  }
}

void TeleportWarmStartSample::Draw(DrawList& draw_list) const {
  draw_list.boxes.push_back(
      {ground_.GetCenter(), ground_.GetHalfSize(), sample_color::kGrey});
  for (std::size_t i = 0; i < box_count_; ++i) {
    const Body& body = world_.body(world_.collider(boxes_[i]).body_index);
    DrawBoxOutline(
        world_.ColliderCenter(boxes_[i]), kBoxHalfSize, body.rotation,
        i == 0 ? sample_color::kOrange : sample_color::kBlue, draw_list);
  }
}

void TeleportWarmStartSample::End() {
  world_.Clear();
  step_ = 0;
  max_spin_ = Scalar{0};
  max_rise_ = Scalar{0};
  moved_y_ = Scalar{0};
  for (auto& readout : readouts_) {
    readout.is_valid = false;
  }
}

}  // namespace neko::tutorial
