#include "tutorials/08_chain/solutions/per_link_stretch_sample.h"

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

void PerLinkStretchSample::Begin(const SampleContext& context) {
  world_ = PhysicsWorld{{.gravity = Vec2f{Scalar{0}, -kGravity},
                         .dt = kFixedDeltaTime,
                         .max_bodies = static_cast<std::size_t>(kMaxLinks + 1),
                         .max_colliders = 1,
                         .max_touching_pairs = 1,
                         .max_broadphase_pairs = 1,
                         .max_joints = static_cast<std::size_t>(kMaxLinks),
                         .velocity_iterations = parameters_[kPasses].value}};
  const Vec2f pivot_position{
      context.world_bounds.GetCenter().x(),
      context.world_bounds.max_bound().y() - Ratio(1, 2)};
  const BodyIndex pivot = world_.AddBody();
  world_.body(pivot).type = BodyType::kStatic;
  world_.body(pivot).inverse_mass = Scalar{0};
  world_.body(pivot).position = pivot_position;
  BodyIndex previous = pivot;
  for (int i = 1; i <= links(); ++i) {
    const BodyIndex link = world_.AddBody();
    world_.body(link).position =
        pivot_position + Vec2f::Down() * (kLinkLength * static_cast<Scalar>(i));
    world_.body(link).inverse_mass =
        i == links()
            ? Scalar{1} / static_cast<Scalar>(parameters_[kEndMass].value)
            : Scalar{1};
    world_.AddDistanceJoint(previous, link);
    previous = link;
  }
  UpdateReadouts();
}

void PerLinkStretchSample::FixedUpdate() {
  world_.Step();
  UpdateReadouts();
}

void PerLinkStretchSample::UpdateReadouts() {
  const int bottom = links();
  const std::array<int, 3> shown = {1, 2, bottom};
  for (std::size_t k = 0; k < shown.size(); ++k) {
    readouts_[2 * k].value = ToDecimal(
        predicted_stretch(shown[k]) * Scalar{1000}, readouts_[2 * k].scale);
    readouts_[2 * k + 1].value = ToDecimal(
        measured_stretch(shown[k]) * Scalar{1000}, readouts_[2 * k + 1].scale);
  }
  Scalar largest{};
  for (int i = 1; i <= bottom; ++i) {
    largest = Max(largest, Abs(measured_stretch(i) - predicted_stretch(i)));
  }
  readouts_[6].value = ToDecimal(largest * Scalar{1000}, readouts_[6].scale);
  for (auto& readout : readouts_) {
    readout.is_valid = true;
  }
}

Scalar PerLinkStretchSample::predicted_stretch(int joint) const {
  const Scalar end_mass = static_cast<Scalar>(parameters_[kEndMass].value);
  const auto inverse_mass = [this, end_mass](int i) {
    return i == links() ? Scalar{1} / end_mass : Scalar{1};
  };
  const Scalar omega = Scalar{2} * Pi<Scalar>() * kJointHertz;
  // Links joint to N - 1 of 1 kg, and the end
  const Scalar load =
      kGravity * (static_cast<Scalar>(links() - joint) + end_mass);
  const Scalar inverse_effective_mass =
      inverse_mass(joint) + (joint > 1 ? inverse_mass(joint - 1) : Scalar{0});
  return load * inverse_effective_mass / (omega * omega);
}

Scalar PerLinkStretchSample::measured_stretch(int joint) const {
  return world_.JointDistance(JointAbove(joint)) - kLinkLength;
}

void PerLinkStretchSample::Draw(DrawList& draw_list) const {
  Scalar largest{};
  for (int i = 1; i <= links(); ++i) {
    largest = Max(largest, predicted_stretch(i));
  }
  for (int i = 1; i <= links(); ++i) {
    // Three shades, by thirds of the largest predicted stretch
    const Scalar share = measured_stretch(i) / largest;
    const ColorId color = share > Ratio(2, 3)   ? sample_color::kRed
                          : share > Ratio(1, 3) ? sample_color::kYellow
                                                : sample_color::kWhite;
    draw_list.lines.push_back({world_.body(Link(i - 1)).position,
                               world_.body(Link(i)).position, color});
  }
  draw_list.circles.push_back(
      {world_.body(Link(0)).position, kLinkRadius, sample_color::kGrey});
  for (int i = 1; i < links(); ++i) {
    draw_list.circles.push_back(
        {world_.body(Link(i)).position, kLinkRadius, sample_color::kOrange});
  }
  draw_list.circles.push_back(
      {world_.body(Link(links())).position, kEndRadius, sample_color::kRed});
}

void PerLinkStretchSample::End() {
  world_.Clear();
  for (auto& readout : readouts_) {
    readout.is_valid = false;
  }
}

}  // namespace neko::tutorial
