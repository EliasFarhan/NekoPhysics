#include "tutorials/08_chain/solutions/hertz_sweep_sample.h"

#include "neko/math/const.h"
#include "neko/math/func.h"

namespace neko::tutorial {
namespace {
constexpr Scalar kGravity = Ratio(981, 100);
constexpr Scalar kLinkLength = Ratio(1, 2);
constexpr Scalar kLinkRadius = Ratio(2, 25);
constexpr Scalar kEndRadius = Ratio(1, 5);
// Past this total stretch the chain blew up: stop stepping
constexpr Scalar kDivergedStretch{10};
constexpr BodyIndex Link(int i) { return BodyIndex{static_cast<Index>(i)}; }
constexpr JointIndex JointAbove(int i) {
  return JointIndex{static_cast<Index>(i - 1)};
}
}  // namespace

void HertzSweepSample::Begin(const SampleContext& context) {
  world_ = PhysicsWorld{
      {.gravity = Vec2f{Scalar{0}, -kGravity},
       .dt = kFixedDeltaTime,
       .max_bodies = static_cast<std::size_t>(kLinks + 1),
       .max_colliders = 1,
       .max_touching_pairs = 1,
       .max_broadphase_pairs = 1,
       .max_joints = static_cast<std::size_t>(kLinks),
       .velocity_iterations = parameters_[kPasses].value,
       .joint_hertz = static_cast<Scalar>(parameters_[kHertz].value),
       .joint_damping_ratio = parameters_[kDamping].ToScalar()}};
  const bool horizontal = parameters_[kStart].value != 0;
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
  for (int i = 1; i <= kLinks; ++i) {
    const BodyIndex link = world_.AddBody();
    world_.body(link).position =
        pivot_position + direction * (kLinkLength * static_cast<Scalar>(i));
    world_.body(link).inverse_mass =
        i == kLinks
            ? Scalar{1} / static_cast<Scalar>(parameters_[kEndMass].value)
            : Scalar{1};
    world_.AddDistanceJoint(previous, link);
    previous = link;
  }
  max_stretch_ = Scalar{0};
  diverged_ = false;
  readouts_[0].value =
      ToDecimal(predicted_rest_stretch() * Scalar{1000}, readouts_[0].scale);
  readouts_[0].is_valid = true;
  UpdateReadouts();
}

void HertzSweepSample::FixedUpdate() {
  if (diverged_) {
    return;
  }
  world_.Step();
  const Scalar now = stretch();
  if (Abs(now) > kDivergedStretch) {
    diverged_ = true;
  }
  max_stretch_ = Max(max_stretch_, now);
  UpdateReadouts();
}

void HertzSweepSample::UpdateReadouts() {
  readouts_[1].value = ToDecimal(stretch() * Scalar{1000}, readouts_[1].scale);
  readouts_[2].value =
      ToDecimal(max_stretch_ * Scalar{1000}, readouts_[2].scale);
  readouts_[3].value = diverged_ ? 1 : 0;
  for (std::size_t i = 1; i < readouts_.size(); ++i) {
    readouts_[i].is_valid = true;
  }
}

Scalar HertzSweepSample::predicted_rest_stretch() const {
  const Scalar end_mass = static_cast<Scalar>(parameters_[kEndMass].value);
  const auto inverse_mass = [end_mass](int i) {
    return i == kLinks ? Scalar{1} / end_mass : Scalar{1};
  };
  const Scalar omega =
      Scalar{2} * Pi<Scalar>() * static_cast<Scalar>(parameters_[kHertz].value);
  Scalar sum{};
  for (int i = 1; i <= kLinks; ++i) {
    const Scalar load = kGravity * (static_cast<Scalar>(kLinks - i) + end_mass);
    const Scalar inverse_effective_mass =
        inverse_mass(i) + (i > 1 ? inverse_mass(i - 1) : Scalar{0});
    sum += load * inverse_effective_mass;
  }
  // Two divisions: w^2 overflows Fixed16 (32767) past 28.8 Hz
  return sum / omega / omega;
}

Scalar HertzSweepSample::stretch() const {
  Scalar sum{};
  for (int i = 1; i <= kLinks; ++i) {
    sum += world_.JointDistance(JointAbove(i)) - kLinkLength;
  }
  return sum;
}

void HertzSweepSample::Draw(DrawList& draw_list) const {
  for (int i = 1; i <= kLinks; ++i) {
    draw_list.lines.push_back({world_.body(Link(i - 1)).position,
                               world_.body(Link(i)).position,
                               sample_color::kWhite});
  }
  draw_list.circles.push_back(
      {world_.body(Link(0)).position, kLinkRadius, sample_color::kGrey});
  for (int i = 1; i < kLinks; ++i) {
    draw_list.circles.push_back(
        {world_.body(Link(i)).position, kLinkRadius, sample_color::kOrange});
  }
  draw_list.circles.push_back(
      {world_.body(Link(kLinks)).position, kEndRadius, sample_color::kRed});
}

void HertzSweepSample::End() {
  world_.Clear();
  max_stretch_ = Scalar{0};
  diverged_ = false;
  for (auto& readout : readouts_) {
    readout.is_valid = false;
  }
}

}  // namespace neko::tutorial
