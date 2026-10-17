#include "tutorials/06_stacking/solutions/impulse_per_box_sample.h"

#include "main/samples/body_draw.h"

namespace neko::tutorial {
namespace {
constexpr Vec2f kGroundHalfSize{Scalar{6}, Ratio(1, 2)};
constexpr Vec2f kBoxHalfSize{Ratio(1, 2), Ratio(1, 2)};
constexpr Scalar kGravity = Ratio(981, 100);
// Length of an impulse bar per box weight m g dt, in meters
constexpr Scalar kBarPerWeight = Ratio(1, 4);

BodyIndex BoxIndex(int box) { return BodyIndex{static_cast<Index>(box + 1)}; }
}  // namespace

void ImpulsePerBoxSample::Begin(const SampleContext& context) {
  world_ = PhysicsWorld{
      {.gravity = Vec2f{Scalar{0}, -kGravity},
       .dt = kFixedDeltaTime,
       .max_bodies = static_cast<std::size_t>(kMaxBoxes + 1),
       .max_colliders = static_cast<std::size_t>(kMaxBoxes + 1),
       .max_touching_pairs = static_cast<std::size_t>(kMaxBoxes),
       .max_broadphase_pairs = static_cast<std::size_t>(2 * kMaxBoxes),
       .velocity_iterations = parameters_[kPasses].value,
       .warm_starting = parameters_[kWarmStart].value != 0}};

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

  for (int i = 0; i < box_count(); ++i) {
    const BodyIndex box = world_.AddBody();
    world_.body(box).position = {
        center.x(), ground_top_ + kBoxHalfSize.y() +
                        static_cast<Scalar>(i) * kBoxHalfSize.y() * Scalar{2}};
    const ColliderIndex collider = world_.AddAabbCollider(box);
    world_.collider(collider).is_trigger = false;
    world_.collider(collider).restitution = Scalar{0};
    world_.aabb(world_.collider(collider).shape_index).half_size = kBoxHalfSize;
  }
  impulses_.fill(Scalar{0});
  first_impulses_.fill(Scalar{0});
  readouts_[0].value = ToDecimal(predicted_impulse(0), readouts_[0].scale);
  readouts_[3].value =
      ToDecimal(predicted_impulse(box_count() - 1), readouts_[3].scale);
  readouts_[0].is_valid = true;
  readouts_[3].is_valid = true;
}

void ImpulsePerBoxSample::FixedUpdate() {
  std::array<Scalar, kMaxBoxes> before{};
  for (int i = 0; i < box_count(); ++i) {
    before[static_cast<std::size_t>(i)] = world_.body(BoxIndex(i)).velocity.y();
  }
  world_.Step();
  ++step_;
  // From the top down: each box's momentum change, plus the weight it gained
  // over the step, plus what the contact above pushed down (m = 1 kg)
  const Scalar weight = kGravity * world_.dt();
  Scalar above{};
  for (int i = box_count() - 1; i >= 0; --i) {
    const auto k = static_cast<std::size_t>(i);
    const Scalar change = world_.body(BoxIndex(i)).velocity.y() - before[k];
    impulses_[k] = change + weight + above;
    above = impulses_[k];
  }
  if (step_ == 1) {
    first_impulses_ = impulses_;
    readouts_[1].value = ToDecimal(first_step_impulse(0), readouts_[1].scale);
    readouts_[4].value =
        ToDecimal(first_step_impulse(box_count() - 1), readouts_[4].scale);
    readouts_[1].is_valid = true;
    readouts_[4].is_valid = true;
  }
  UpdateReadouts();
}

void ImpulsePerBoxSample::UpdateReadouts() {
  readouts_[2].value = ToDecimal(measured_impulse(0), readouts_[2].scale);
  readouts_[5].value =
      ToDecimal(measured_impulse(box_count() - 1), readouts_[5].scale);
  readouts_[6].value = ToDecimal(top_sink(), readouts_[6].scale);
  readouts_[2].is_valid = true;
  readouts_[5].is_valid = true;
  readouts_[6].is_valid = true;
}

Scalar ImpulsePerBoxSample::predicted_impulse(int contact) const {
  return static_cast<Scalar>(box_count() - contact) * kGravity *
         world_.dt();  // m = 1 kg
}

Scalar ImpulsePerBoxSample::top_sink() const {
  const Body& top = world_.body(BoxIndex(box_count() - 1));
  const Scalar perfect =
      ground_top_ + kBoxHalfSize.y() +
      static_cast<Scalar>(box_count() - 1) * kBoxHalfSize.y() * Scalar{2};
  return perfect - top.position.y();
}

void ImpulsePerBoxSample::Draw(DrawList& draw_list) const {
  const Body& ground = world_.body(BodyIndex{0});
  draw_list.boxes.push_back(
      {ground.position, kGroundHalfSize, sample_color::kDarkGrey});
  const Scalar weight = kGravity * world_.dt();
  const Scalar bar_start = center_x_ + kBoxHalfSize.x() + Ratio(1, 10);
  for (int i = 0; i < box_count(); ++i) {
    const Body& box = world_.body(BoxIndex(i));
    DrawBoxOutline(box.position, kBoxHalfSize, box.rotation,
                   sample_color::kOrange, draw_list);
    // The contact under box i, at the box's bottom face
    const Scalar y = box.position.y() - kBoxHalfSize.y();
    const Scalar measured = measured_impulse(i) / weight * kBarPerWeight;
    const Scalar predicted = predicted_impulse(i) / weight * kBarPerWeight;
    draw_list.lines.push_back(
        {{bar_start, y}, {bar_start + measured, y}, sample_color::kRed});
    draw_list.lines.push_back({{bar_start + predicted, y - Ratio(1, 10)},
                               {bar_start + predicted, y + Ratio(1, 10)},
                               sample_color::kWhite});
  }
}

void ImpulsePerBoxSample::End() {
  world_.Clear();
  step_ = 0;
  for (const std::size_t i : {1u, 2u, 4u, 5u, 6u}) {
    readouts_[i].is_valid = false;
  }
}

}  // namespace neko::tutorial
