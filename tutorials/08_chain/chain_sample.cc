#include "tutorials/08_chain/chain_sample.h"

#include "neko/math/const.h"
#include "neko/math/func.h"

namespace neko::tutorial {
namespace {
constexpr Scalar kGravity = Ratio(981, 100);
// PhysicsWorldConfig::joint_hertz, the default
constexpr Scalar kJointHertz{15};
constexpr Scalar kLinkLength = Ratio(1, 2);
constexpr Scalar kLinkRadius = Ratio(2, 25);
constexpr Scalar kEndRadius = Ratio(1, 5);
// Body 0 is the pivot, bodies 1 to N the links, joint i - 1 holds link i
constexpr BodyIndex Link(int i) { return BodyIndex{static_cast<Index>(i)}; }
constexpr JointIndex JointAbove(int i) {
  return JointIndex{static_cast<Index>(i - 1)};
}
}  // namespace

void ChainSample::Begin(const SampleContext& context) {
  const int links = parameters_[kLinks].value;
  world_ = PhysicsWorld{{.gravity = Vec2f{Scalar{0}, -kGravity},
                         .dt = kFixedDeltaTime,
                         .max_bodies = static_cast<std::size_t>(kMaxLinks + 1),
                         .max_colliders = 1,
                         .max_touching_pairs = 1,
                         .max_broadphase_pairs = 1,
                         .max_joints = static_cast<std::size_t>(kMaxLinks),
                         .velocity_iterations = parameters_[kPasses].value}};
  const bool horizontal = parameters_[kStart].value != 0;
  // Hanging: from the top center. Stretched out: from the left, so the chain
  // swings across the screen.
  const Vec2f pivot_position =
      horizontal ? Vec2f{context.world_bounds.min_bound().x() + Scalar{1},
                         context.world_bounds.max_bound().y() - Scalar{1}}
                 : Vec2f{context.world_bounds.GetCenter().x(),
                         context.world_bounds.max_bound().y() - Ratio(1, 2)};
  const Vec2f direction = horizontal ? Vec2f::Right() : Vec2f::Down();

  const BodyIndex pivot = world_.AddBody();
  world_.body(pivot).type = BodyType::kStatic;
  world_.body(pivot).inverse_mass = Scalar{0};
  world_.body(pivot).position = pivot_position;
  BodyIndex previous = pivot;
  for (int i = 1; i <= links; ++i) {
    const BodyIndex link = world_.AddBody();
    world_.body(link).position =
        pivot_position + direction * (kLinkLength * static_cast<Scalar>(i));
    world_.body(link).inverse_mass =
        i == links
            ? Scalar{1} / static_cast<Scalar>(parameters_[kEndMass].value)
            : Scalar{1};
    world_.AddDistanceJoint(previous, link);
    previous = link;
  }
  max_stretch_ = Scalar{0};
  readouts_[0].value =
      ToDecimal(predicted_rest_stretch() * Scalar{1000}, readouts_[0].scale);
  readouts_[0].is_valid = true;
  UpdateReadouts();
}

void ChainSample::FixedUpdate() {
  world_.Step();
  max_stretch_ = Max(max_stretch_, stretch());
  UpdateReadouts();
}

void ChainSample::UpdateReadouts() {
  readouts_[1].value = ToDecimal(stretch() * Scalar{1000}, readouts_[1].scale);
  readouts_[2].value =
      ToDecimal(max_stretch_ * Scalar{1000}, readouts_[2].scale);
  readouts_[1].is_valid = true;
  readouts_[2].is_valid = true;
}

void ChainSample::Draw(DrawList& draw_list) const {
  const int links = parameters_[kLinks].value;
  for (int i = 1; i <= links; ++i) {
    draw_list.lines.push_back({world_.body(Link(i - 1)).position,
                               world_.body(Link(i)).position,
                               sample_color::kWhite});
  }
  draw_list.circles.push_back(
      {world_.body(Link(0)).position, kLinkRadius, sample_color::kGrey});
  for (int i = 1; i < links; ++i) {
    draw_list.circles.push_back(
        {world_.body(Link(i)).position, kLinkRadius, sample_color::kOrange});
  }
  draw_list.circles.push_back(
      {world_.body(Link(links)).position, kEndRadius, sample_color::kRed});
}

void ChainSample::End() {
  world_.Clear();
  max_stretch_ = Scalar{0};
  for (auto& readout : readouts_) {
    readout.is_valid = false;
  }
}

Scalar ChainSample::predicted_rest_stretch() const {
  const int links = parameters_[kLinks].value;
  const Scalar end_mass = static_cast<Scalar>(parameters_[kEndMass].value);
  const auto inverse_mass = [links, end_mass](int i) {
    return i == links ? Scalar{1} / end_mass : Scalar{1};
  };
  const Scalar omega = Scalar{2} * Pi<Scalar>() * kJointHertz;
  Scalar sum{};
  for (int i = 1; i <= links; ++i) {
    // Everything below the joint: links i to N - 1 of 1 kg, and the end
    const Scalar load = kGravity * (static_cast<Scalar>(links - i) + end_mass);
    const Scalar inverse_effective_mass =
        inverse_mass(i) + (i > 1 ? inverse_mass(i - 1) : Scalar{0});
    sum += load * inverse_effective_mass;
  }
  return sum / (omega * omega);
}

Scalar ChainSample::stretch() const {
  Scalar sum{};
  for (int i = 1; i <= parameters_[kLinks].value; ++i) {
    sum += world_.JointDistance(JointAbove(i)) - kLinkLength;
  }
  return sum;
}

}  // namespace neko::tutorial
