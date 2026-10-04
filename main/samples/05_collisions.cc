#include "main/samples/05_collisions.h"

#include "main/samples/sample_rng.h"

namespace neko {

constexpr static Scalar kMaxSpeed = Scalar{4};
constexpr static Scalar kMaxCircleRadius = Ratio(1, 5);
constexpr static Scalar kMinCircleRadius = Ratio(1, 10);
constexpr static ColorId kCollidingColor = sample_color::kGreen;
constexpr static ColorId kNotCollidingColor = sample_color::kRed;
constexpr static std::uint32_t kSeed = 0x05C0FFEEu;

void CollisionsSample::Begin(const SampleContext& context) {
  bounds_ = context.world_bounds;
  world_.SetContactListener(this);
  world_.SetBSH(&quad_tree_);
  SampleRng rng{kSeed};
  bodies_.resize(kCircleCount);
  for (Index i = 0; i < kCircleCount; ++i) {
    const auto index = world_.AddBody();
    Body& body = world_.body(index);
    body.position = {
        rng.Range(bounds_.min_bound().x(), bounds_.max_bound().x()),
        rng.Range(bounds_.min_bound().y(), bounds_.max_bound().y())};
    body.velocity = Vec2f{rng.Range(Scalar{-1}, Scalar{1}),
                          rng.Range(Scalar{-1}, Scalar{1})} *
                    kMaxSpeed;
    body.inverse_mass = Scalar{1};

    auto& colliding_body = bodies_[i];
    colliding_body.circle_radius =
        rng.Range(kMinCircleRadius, kMaxCircleRadius);
    const auto collider_index = world_.AddCircleCollider(index);
    auto& collider = world_.collider(collider_index);
    collider.is_trigger = false;
    world_.circle(collider.shape_index).radius = colliding_body.circle_radius;
  }
}

void CollisionsSample::Draw(DrawList& draw_list) const {
  for (Index i = 0; i < bodies_.size(); ++i) {
    const auto& colliding_body = bodies_[i];
    draw_list.circles.push_back(
        {world_.body(BodyIndex{i}).position, colliding_body.circle_radius,
         colliding_body.count == 0 ? kNotCollidingColor : kCollidingColor});
  }
}

void CollisionsSample::End() {
  world_.Clear();
  bodies_.clear();
  quad_tree_.Clear();
}

void CollisionsSample::FixedUpdate() {
  world_.Step();
  for (Index i = 0; i < bodies_.size(); ++i) {
    BounceInside(world_.body(BodyIndex{i}), bounds_);
  }
}

void CollisionsSample::OnCollisionEnter(const ColliderPair& p) {
  bodies_[world_.collider(p.c1()).body_index.index()].count++;
  bodies_[world_.collider(p.c2()).body_index.index()].count++;
}

void CollisionsSample::OnCollisionExit(const ColliderPair& p) {
  bodies_[world_.collider(p.c1()).body_index.index()].count--;
  bodies_[world_.collider(p.c2()).body_index.index()].count--;
}

void CollisionsSample::OnTriggerEnter([[maybe_unused]] const ColliderPair& p) {}

void CollisionsSample::OnTriggerExit([[maybe_unused]] const ColliderPair& p) {}
}  // namespace neko
