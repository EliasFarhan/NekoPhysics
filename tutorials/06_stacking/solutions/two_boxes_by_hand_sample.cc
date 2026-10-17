#include "tutorials/06_stacking/solutions/two_boxes_by_hand_sample.h"

#include "neko/math/func.h"

#include "main/samples/body_draw.h"

namespace neko::tutorial {
namespace {
constexpr Vec2f kGroundHalfSize{Scalar{6}, Ratio(1, 2)};
constexpr Vec2f kBoxHalfSize{Ratio(1, 2), Ratio(1, 2)};
constexpr Scalar kGravity = Ratio(981, 100);
constexpr BodyIndex kBottomBox{1};
constexpr BodyIndex kTopBox{2};
}  // namespace

void TwoBoxesByHandSample::Begin(const SampleContext& context) {
  world_ = PhysicsWorld{{.gravity = Vec2f{Scalar{0}, -kGravity},
                         .dt = kFixedDeltaTime,
                         .max_bodies = 3,
                         .max_colliders = 3,
                         .max_touching_pairs = 2,
                         .max_broadphase_pairs = 4,
                         .velocity_iterations = parameters_[kPasses].value}};
  const Vec2f center = context.world_bounds.GetCenter();
  const Scalar bottom = context.world_bounds.min_bound().y();
  const BodyIndex ground = world_.AddBody();
  world_.body(ground).type = BodyType::kStatic;
  world_.body(ground).inverse_mass = Scalar{0};
  world_.body(ground).position = {center.x(), bottom + kGroundHalfSize.y()};
  const ColliderIndex ground_collider = world_.AddAabbCollider(ground);
  world_.collider(ground_collider).is_trigger = false;
  world_.collider(ground_collider).restitution = Scalar{0};
  world_.aabb(world_.collider(ground_collider).shape_index).half_size =
      kGroundHalfSize;
  const Scalar ground_top = bottom + kGroundHalfSize.y() * Scalar{2};
  for (int i = 0; i < 2; ++i) {
    const BodyIndex box = world_.AddBody();
    world_.body(box).position = {
        center.x(), ground_top + kBoxHalfSize.y() +
                        static_cast<Scalar>(i) * kBoxHalfSize.y() * Scalar{2}};
    const ColliderIndex collider = world_.AddAabbCollider(box);
    world_.collider(collider).is_trigger = false;
    world_.collider(collider).restitution = Scalar{0};
    world_.aabb(world_.collider(collider).shape_index).half_size = kBoxHalfSize;
  }

  const State hand = predicted(parameters_[kPasses].value);
  readouts_[0].value = ToDecimal(hand.bottom_velocity, readouts_[0].scale);
  readouts_[2].value = ToDecimal(hand.top_velocity, readouts_[2].scale);
  readouts_[4].value = ToDecimal(hand.bottom_impulse, readouts_[4].scale);
  readouts_[6].value = ToDecimal(hand.top_impulse, readouts_[6].scale);
  for (const std::size_t i : {0u, 2u, 4u, 6u}) {
    readouts_[i].is_valid = true;
  }
}

void TwoBoxesByHandSample::FixedUpdate() {
  const Scalar bottom_before = world_.body(kBottomBox).velocity.y();
  const Scalar top_before = world_.body(kTopBox).velocity.y();
  world_.Step();
  if (measured_.has_value()) {
    return;
  }
  // Momentum changes over the step (m = 1 kg): the top box got B and its
  // weight, the bottom box A, -B and its weight
  const Scalar weight = kGravity * world_.dt();
  State state{};
  state.bottom_velocity = world_.body(kBottomBox).velocity.y();
  state.top_velocity = world_.body(kTopBox).velocity.y();
  state.top_impulse = state.top_velocity - top_before + weight;
  state.bottom_impulse =
      state.bottom_velocity - bottom_before + weight + state.top_impulse;
  measured_ = state;
  readouts_[1].value = ToDecimal(state.bottom_velocity, readouts_[1].scale);
  readouts_[3].value = ToDecimal(state.top_velocity, readouts_[3].scale);
  readouts_[5].value = ToDecimal(state.bottom_impulse, readouts_[5].scale);
  readouts_[7].value = ToDecimal(state.top_impulse, readouts_[7].scale);
  for (const std::size_t i : {1u, 3u, 5u, 7u}) {
    readouts_[i].is_valid = true;
  }
}

TwoBoxesByHandSample::State TwoBoxesByHandSample::predicted(int passes) const {
  // After gravity, before the solver
  State state{};
  state.bottom_velocity = -kGravity * world_.dt();
  state.top_velocity = state.bottom_velocity;
  for (int pass = 0; pass < passes; ++pass) {
    // Contact A, two points: k = 1 per kg, target 0, the total stays >= 0
    for (int point = 0; point < 2; ++point) {
      const Scalar total =
          Max(state.bottom_impulse - state.bottom_velocity, Scalar{0});
      state.bottom_velocity += total - state.bottom_impulse;
      state.bottom_impulse = total;
    }
    // Contact B, two points: k = 2 per kg
    for (int point = 0; point < 2; ++point) {
      const Scalar approach = state.top_velocity - state.bottom_velocity;
      const Scalar total =
          Max(state.top_impulse - approach / Scalar{2}, Scalar{0});
      const Scalar impulse = total - state.top_impulse;
      state.top_velocity += impulse;
      state.bottom_velocity -= impulse;
      state.top_impulse = total;
    }
  }
  return state;
}

void TwoBoxesByHandSample::Draw(DrawList& draw_list) const {
  const Body& ground = world_.body(BodyIndex{0});
  draw_list.boxes.push_back(
      {ground.position, kGroundHalfSize, sample_color::kDarkGrey});
  for (const BodyIndex index : {kBottomBox, kTopBox}) {
    const Body& box = world_.body(index);
    DrawBoxOutline(box.position, kBoxHalfSize, box.rotation,
                   sample_color::kOrange, draw_list);
    DrawVelocity(box, sample_color::kSky, draw_list);
  }
}

void TwoBoxesByHandSample::End() {
  world_.Clear();
  measured_.reset();
  for (const std::size_t i : {1u, 3u, 5u, 7u}) {
    readouts_[i].is_valid = false;
  }
}

}  // namespace neko::tutorial
