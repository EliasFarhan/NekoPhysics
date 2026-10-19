#include "tutorials/16_seesaw/solutions/retarget_chain_sample.h"

#include "neko/math/const.h"
#include "neko/math/func.h"

#include "main/samples/body_draw.h"
#include "tutorials/16_seesaw/seesaw_sample.h"

namespace neko::tutorial {
namespace {
constexpr Vec2f kGroundHalfSize{Scalar{8}, Ratio(1, 2)};
constexpr Vec2f kPlayerHalfSize{Ratio(2, 5), Ratio(2, 5)};
constexpr Scalar kPlayerMass{1};
// The players stand this far left and right of the post
constexpr Scalar kPlayerOffset = Ratio(3, 2);
// The post, above the ground's top
constexpr Scalar kPostHeight{4};
constexpr Scalar kLinkRadius = Ratio(1, 10);
constexpr Scalar kLinkInverseMass{5};
constexpr Scalar kFriction = Ratio(2, 5);
constexpr Scalar kPostRadius = Ratio(1, 10);
}  // namespace

void RetargetChainSample::Begin(const SampleContext& context) {
  const Scalar center_x = context.world_bounds.GetCenter().x();
  const Scalar bottom = context.world_bounds.min_bound().y();
  ground_ = AddAxle(world_, {center_x, bottom + kGroundHalfSize.y()});
  {
    const ColliderIndex index = world_.AddAabbCollider(ground_);
    Collider& collider = world_.collider(index);
    collider.is_trigger = false;
    collider.restitution = Scalar{0};
    collider.friction = kFriction;
    world_.aabb(collider.shape_index).half_size = kGroundHalfSize;
  }
  const Scalar ground_top = bottom + Scalar{2} * kGroundHalfSize.y();
  for (std::size_t i = 0; i < 2; ++i) {
    const Scalar side = i == 0 ? Scalar{-1} : Scalar{1};
    players_[i] = AddPlank(
        world_,
        {center_x + side * kPlayerOffset, ground_top + kPlayerHalfSize.y()},
        kPlayerHalfSize, Scalar{1} / kPlayerMass, kFriction);
  }
  const Vec2f post{center_x, ground_top + kPostHeight};
  post_ = AddAxle(world_, post);

  // The links on the straight line from the post to player A's center, the
  // ropes just taut between them
  const Vec2f end = world_.body(players_[0]).position;
  BodyIndex previous = post_;
  for (int i = 0; i < kLinkCount; ++i) {
    const Scalar t =
        static_cast<Scalar>(i + 1) / static_cast<Scalar>(kLinkCount + 1);
    const BodyIndex link = world_.AddBody();
    world_.body(link).position = post + (end - post) * t;
    world_.body(link).inverse_mass = kLinkInverseMass;
    const ColliderIndex index = world_.AddCircleCollider(link);
    Collider& collider = world_.collider(index);
    collider.is_trigger = false;
    collider.restitution = Scalar{0};
    collider.friction = kFriction;
    world_.circle(collider.shape_index).radius = kLinkRadius;
    links_[static_cast<std::size_t>(i)] = link;
    end_link_collider_ = index;
    const JointIndex rope = world_.AddDistanceJoint(previous, link);
    world_.joint(rope).type = DistanceJointType::kRope;
    ropes_[static_cast<std::size_t>(i)] = rope;
    previous = link;
  }
  const JointIndex last = world_.AddDistanceJoint(previous, players_[0]);
  world_.joint(last).type = DistanceJointType::kRope;
  ropes_[kLinkCount] = last;
  world_.SetContactListener(this);
}

void RetargetChainSample::FixedUpdate() {
  if (step_ > 0 && step_ % kRetargetPeriod == 0) {
    const JointIndex rope = ropes_[kLinkCount];
    attached_ = 1 - attached_;
    world_.SetJointBodies(rope, links_[kLinkCount - 1], players_[attached_]);
    if (parameters_[kKeepLength].value == 0) {
      world_.joint(rope).length = world_.JointDistance(rope);
    }
    ++retargets_;
    velocity_before_ = world_.body(players_[attached_]).velocity;
    is_measuring_kick_ = true;
  }
  world_.Step();
  ++step_;
  if (is_measuring_kick_) {
    max_kick_ = Max(max_kick_, (world_.body(players_[attached_]).velocity -
                                velocity_before_)
                                   .Length());
    is_measuring_kick_ = false;
  }
  readouts_[0].value = retargets_;
  readouts_[1].value = enters_;
  readouts_[2].value = exits_;
  readouts_[3].value = ToDecimal(max_kick_, readouts_[3].scale);
  for (auto& readout : readouts_) {
    readout.is_valid = true;
  }
}

void RetargetChainSample::Draw(DrawList& draw_list) const {
  const Body& ground = world_.body(ground_);
  DrawBoxOutline(ground.position, kGroundHalfSize, ground.rotation,
                 sample_color::kGrey, draw_list);
  draw_list.circles.push_back(
      {world_.body(post_).position, kPostRadius, sample_color::kGrey});
  for (const JointIndex rope : ropes_) {
    const auto anchors = world_.JointAnchors(rope);
    draw_list.lines.push_back({anchors[0], anchors[1], sample_color::kYellow});
  }
  for (const BodyIndex link : links_) {
    draw_list.circles.push_back(
        {world_.body(link).position, kLinkRadius, sample_color::kYellow});
  }
  for (std::size_t i = 0; i < 2; ++i) {
    const Body& player = world_.body(players_[i]);
    DrawBoxOutline(player.position, kPlayerHalfSize, player.rotation,
                   i == attached_ ? sample_color::kOrange : sample_color::kSky,
                   draw_list);
  }
}

void RetargetChainSample::End() {
  world_.SetContactListener(nullptr);
  world_.Clear();
  attached_ = 0;
  step_ = 0;
  retargets_ = 0;
  enters_ = 0;
  exits_ = 0;
  velocity_before_ = {};
  is_measuring_kick_ = false;
  max_kick_ = Scalar{0};
  for (auto& readout : readouts_) {
    readout.is_valid = false;
  }
}

bool RetargetChainSample::IsEndLinkPair(const ColliderPair& pair) const {
  if (!pair.Contains(end_link_collider_)) {
    return false;
  }
  const ColliderIndex other =
      pair.c1() == end_link_collider_ ? pair.c2() : pair.c1();
  const BodyIndex body = world_.collider(other).body_index;
  return body == players_[0] || body == players_[1];
}

void RetargetChainSample::OnTriggerEnter(
    [[maybe_unused]] const ColliderPair& pair) {}

void RetargetChainSample::OnTriggerExit(
    [[maybe_unused]] const ColliderPair& pair) {}

void RetargetChainSample::OnCollisionEnter(const ColliderPair& pair) {
  if (IsEndLinkPair(pair)) {
    ++enters_;
  }
}

void RetargetChainSample::OnCollisionExit(const ColliderPair& pair) {
  if (IsEndLinkPair(pair)) {
    ++exits_;
  }
}

}  // namespace neko::tutorial
