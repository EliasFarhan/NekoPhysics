#include "tutorials/06_stacking/stacking_sample.h"

#include "main/samples/body_draw.h"

namespace neko::tutorial {
namespace {
constexpr Vec2f kGroundHalfSize{Scalar{6}, Ratio(1, 2)};
constexpr Vec2f kBoxHalfSize{Ratio(1, 2), Ratio(1, 2)};
}  // namespace

void StackingSample::Begin(const SampleContext& context) {
  const int boxes = parameters_[kBoxes].value;
  world_ = PhysicsWorld{
      {.gravity = Vec2f{Scalar{0}, -Ratio(981, 100)},
       .dt = kFixedDeltaTime,
       .max_bodies = static_cast<std::size_t>(kMaxBoxes + 1),
       .max_colliders = static_cast<std::size_t>(kMaxBoxes + 1),
       .max_touching_pairs = static_cast<std::size_t>(kMaxBoxes),
       .max_broadphase_pairs = static_cast<std::size_t>(2 * kMaxBoxes),
       .velocity_iterations = parameters_[kPasses].value,
       .warm_starting = parameters_[kWarmStart].value != 0}};
  const bool oriented = parameters_[kOriented].value != 0;

  const Vec2f center = context.world_bounds.GetCenter();
  const Scalar bottom = context.world_bounds.min_bound().y();
  center_x_ = center.x();
  const BodyIndex ground = world_.AddBody();
  world_.body(ground).type = BodyType::kStatic;
  world_.body(ground).inverse_mass = Scalar{0};
  world_.body(ground).position = {center.x(), bottom + kGroundHalfSize.y()};
  ground_top_ = bottom + kGroundHalfSize.y() * Scalar{2};
  const ColliderIndex ground_collider = world_.AddAabbCollider(ground);
  world_.collider(ground_collider).is_trigger = false;
  world_.collider(ground_collider).restitution = Scalar{0};
  world_.aabb(world_.collider(ground_collider).shape_index).half_size =
      kGroundHalfSize;

  for (int i = 0; i < boxes; ++i) {
    const BodyIndex box = world_.AddBody();
    world_.body(box).position = {
        center.x(), ground_top_ + kBoxHalfSize.y() +
                        static_cast<Scalar>(i) * kBoxHalfSize.y() * Scalar{2}};
    const ColliderIndex collider = oriented ? world_.AddOrientedBoxCollider(box)
                                            : world_.AddAabbCollider(box);
    world_.collider(collider).is_trigger = false;
    world_.collider(collider).restitution = Scalar{0};
    if (oriented) {
      world_.oriented_box(world_.collider(collider).shape_index).half_size =
          kBoxHalfSize;
    } else {
      world_.aabb(world_.collider(collider).shape_index).half_size =
          kBoxHalfSize;
    }
  }
  UpdateReadouts();
}

void StackingSample::FixedUpdate() {
  world_.Step();
  UpdateReadouts();
}

void StackingSample::UpdateReadouts() {
  readouts_[0].value = ToDecimal(top_sink(), readouts_[0].scale);
  readouts_[1].value = ToDecimal(drift(), readouts_[1].scale);
  readouts_[0].is_valid = true;
  readouts_[1].is_valid = true;
}

void StackingSample::Draw(DrawList& draw_list) const {
  const Body& ground = world_.body(BodyIndex{0});
  draw_list.boxes.push_back(
      {ground.position, kGroundHalfSize, sample_color::kDarkGrey});
  for (int i = 0; i < parameters_[kBoxes].value; ++i) {
    const Body& box = world_.body(BodyIndex{static_cast<Index>(i + 1)});
    DrawBoxOutline(box.position, kBoxHalfSize, box.rotation,
                   sample_color::kOrange, draw_list);
  }
}

void StackingSample::End() { world_.Clear(); }

Scalar StackingSample::top_sink() const {
  const int boxes = parameters_[kBoxes].value;
  const Body& top = world_.body(BodyIndex{static_cast<Index>(boxes)});
  const Scalar perfect =
      ground_top_ + kBoxHalfSize.y() +
      static_cast<Scalar>(boxes - 1) * kBoxHalfSize.y() * Scalar{2};
  return perfect - top.position.y();
}

Scalar StackingSample::drift() const {
  Scalar largest{};
  for (int i = 0; i < parameters_[kBoxes].value; ++i) {
    const Body& box = world_.body(BodyIndex{static_cast<Index>(i + 1)});
    largest = Max(largest, Abs(box.position.x() - center_x_));
  }
  return largest;
}

}  // namespace neko::tutorial
