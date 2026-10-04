#include "main/samples/04_ground.h"

#include "neko/core/profile.h"

namespace neko {

constexpr static Scalar kCircleRadius = Ratio(1, 2);
constexpr static Vec2f kGroundHalfSize = Vec2f{Scalar{5}, Scalar{1}};
constexpr static Vec2f kRectHalfSize = Vec2f{Ratio(1, 2), Ratio(1, 2)};
constexpr static ColorId kGroundColor = sample_color::kGreen;
constexpr static ColorId kCircleColor = sample_color::kBlue;
constexpr static ColorId kCollisionCircleColor = sample_color::kMagenta;
constexpr static ColorId kRectColor = sample_color::kWhite;

void GroundSample::Begin(const SampleContext& context) {
  world_.SetContactListener(this);
  world_.SetBSH(&quad_tree_);
  const auto world_center = context.world_bounds.GetCenter();

  // Implement ground
  {
    ground_body_index_ = world_.AddBody();
    Body& body = world_.body(ground_body_index_);
    body.position = world_center + Vec2f{Scalar{0}, Scalar{3}};
    body.type = BodyType::kStatic;
    body.inverse_mass = Scalar{0};

    const auto collider_index = world_.AddAabbCollider(ground_body_index_);
    auto& collider = world_.collider(collider_index);
    world_.aabb(collider.shape_index).half_size = kGroundHalfSize;
    collider.is_trigger = false;
    collider.restitution = Scalar{0};
  }

  // Implement ball
  {
    circle_body_index_ = world_.AddBody();
    Body& body = world_.body(circle_body_index_);
    body.position = world_center - Vec2f{Scalar{2}, Scalar{3}};
    body.type = BodyType::kDynamic;
    body.inverse_mass = Scalar{1};

    const auto collider_index = world_.AddCircleCollider(circle_body_index_);
    auto& collider = world_.collider(collider_index);
    world_.circle(collider.shape_index).radius = kCircleRadius;
    collider.is_trigger = false;
    collider.restitution = Ratio(1, 2);
  }

  // Implement box
  {
    rect_body_index_ = world_.AddBody();
    Body& body = world_.body(rect_body_index_);
    body.position = world_center - Vec2f{Scalar{-2}, Scalar{3}};
    body.type = BodyType::kDynamic;
    body.inverse_mass = Scalar{1};

    const auto collider_index = world_.AddAabbCollider(rect_body_index_);
    auto& collider = world_.collider(collider_index);
    world_.aabb(collider.shape_index).half_size = kRectHalfSize;
    collider.is_trigger = false;
    collider.restitution = Scalar{0};
  }
}

void GroundSample::Draw(DrawList& draw_list) const {
  draw_list.circles.push_back(
      {world_.body(circle_body_index_).position, kCircleRadius,
       collision_count_ > 0 ? kCollisionCircleColor : kCircleColor});
  draw_list.boxes.push_back({world_.body(ground_body_index_).position,
                             kGroundHalfSize, kGroundColor});
  draw_list.boxes.push_back(
      {world_.body(rect_body_index_).position, kRectHalfSize, kRectColor});
}

void GroundSample::End() {
  world_.Clear();
  quad_tree_.Clear();
  collision_count_ = 0;
}

void GroundSample::OnTriggerEnter([[maybe_unused]] const ColliderPair& p) {}

void GroundSample::OnTriggerExit([[maybe_unused]] const ColliderPair& p) {}

void GroundSample::FixedUpdate() {
  NEKO_PROFILE_FUNCTION();
  world_.Step();
}

void GroundSample::OnCollisionEnter([[maybe_unused]] const ColliderPair& p) {
  collision_count_++;
}

void GroundSample::OnCollisionExit([[maybe_unused]] const ColliderPair& p) {
  collision_count_--;
}
}  // namespace neko
