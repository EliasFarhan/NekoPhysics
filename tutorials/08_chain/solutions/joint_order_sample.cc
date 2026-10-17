#include "tutorials/08_chain/solutions/joint_order_sample.h"

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
constexpr BodyIndex Link(int i) { return BodyIndex{static_cast<Index>(i)}; }

PhysicsWorld MakeWorld(int passes, bool warm_start) {
  const auto bodies = static_cast<std::size_t>(JointOrderSample::kMaxLinks);
  return PhysicsWorld{{.gravity = Vec2f{Scalar{0}, -kGravity},
                       .dt = Sample::kFixedDeltaTime,
                       .max_bodies = bodies + 1,
                       .max_colliders = 1,
                       .max_touching_pairs = 1,
                       .max_broadphase_pairs = 1,
                       .max_joints = bodies,
                       .velocity_iterations = passes,
                       .warm_starting = warm_start}};
}

// The pivot (body 0) and the links (bodies 1 to N), then the joints: from
// the pivot down, or from the end up
void BuildChain(PhysicsWorld& world, Vec2f pivot_position, Vec2f direction,
                int links, Scalar end_mass, bool bottom_up) {
  const BodyIndex pivot = world.AddBody();
  world.body(pivot).type = BodyType::kStatic;
  world.body(pivot).inverse_mass = Scalar{0};
  world.body(pivot).position = pivot_position;
  for (int i = 1; i <= links; ++i) {
    const BodyIndex link = world.AddBody();
    world.body(link).position =
        pivot_position + direction * (kLinkLength * static_cast<Scalar>(i));
    world.body(link).inverse_mass =
        i == links ? Scalar{1} / end_mass : Scalar{1};
  }
  for (int k = 1; k <= links; ++k) {
    const int i = bottom_up ? links + 1 - k : k;
    world.AddDistanceJoint(Link(i - 1), Link(i));
  }
}

Scalar Stretch(const PhysicsWorld& world, int links) {
  Scalar sum{};
  for (int j = 0; j < links; ++j) {
    sum += world.JointDistance(JointIndex{static_cast<Index>(j)}) - kLinkLength;
  }
  return sum;
}

void DrawChain(const PhysicsWorld& world, int links, ColorId joint_color,
               DrawList& draw_list) {
  for (int i = 1; i <= links; ++i) {
    draw_list.lines.push_back({world.body(Link(i - 1)).position,
                               world.body(Link(i)).position, joint_color});
  }
  draw_list.circles.push_back(
      {world.body(Link(0)).position, kLinkRadius, sample_color::kGrey});
  for (int i = 1; i < links; ++i) {
    draw_list.circles.push_back(
        {world.body(Link(i)).position, kLinkRadius, sample_color::kOrange});
  }
  draw_list.circles.push_back(
      {world.body(Link(links)).position, kEndRadius, sample_color::kRed});
}
}  // namespace

void JointOrderSample::Begin(const SampleContext& context) {
  const int passes = parameters_[kPasses].value;
  const bool warm_start = parameters_[kWarmStart].value != 0;
  top_down_ = MakeWorld(passes, warm_start);
  bottom_up_ = MakeWorld(passes, warm_start);
  const bool horizontal = parameters_[kStart].value != 0;
  const Scalar end_mass = static_cast<Scalar>(parameters_[kEndMass].value);
  const Scalar top = context.world_bounds.max_bound().y() -
                     (horizontal ? Scalar{1} : Ratio(1, 2));
  // Hanging: side by side. Stretched out: both from the left, one above the
  // other would overlap, so the bottom-up chain swings from the center
  const Scalar left = horizontal
                          ? context.world_bounds.min_bound().x() + Scalar{1}
                          : context.world_bounds.GetCenter().x() - Scalar{2};
  const Scalar right = horizontal
                           ? context.world_bounds.GetCenter().x()
                           : context.world_bounds.GetCenter().x() + Scalar{2};
  const Vec2f direction = horizontal ? Vec2f::Right() : Vec2f::Down();
  BuildChain(top_down_, {left, top}, direction, links(), end_mass, false);
  BuildChain(bottom_up_, {right, top}, direction, links(), end_mass, true);
  top_down_max_ = Scalar{0};
  bottom_up_max_ = Scalar{0};
  readouts_[0].value =
      ToDecimal(predicted_rest_stretch() * Scalar{1000}, readouts_[0].scale);
  readouts_[0].is_valid = true;
  UpdateReadouts();
}

void JointOrderSample::FixedUpdate() {
  top_down_.Step();
  bottom_up_.Step();
  top_down_max_ = Max(top_down_max_, top_down_stretch());
  bottom_up_max_ = Max(bottom_up_max_, bottom_up_stretch());
  UpdateReadouts();
}

void JointOrderSample::UpdateReadouts() {
  readouts_[1].value =
      ToDecimal(top_down_stretch() * Scalar{1000}, readouts_[1].scale);
  readouts_[2].value =
      ToDecimal(top_down_max_ * Scalar{1000}, readouts_[2].scale);
  readouts_[3].value =
      ToDecimal(bottom_up_stretch() * Scalar{1000}, readouts_[3].scale);
  readouts_[4].value =
      ToDecimal(bottom_up_max_ * Scalar{1000}, readouts_[4].scale);
  for (std::size_t i = 1; i < readouts_.size(); ++i) {
    readouts_[i].is_valid = true;
  }
}

Scalar JointOrderSample::predicted_rest_stretch() const {
  const Scalar end_mass = static_cast<Scalar>(parameters_[kEndMass].value);
  const auto inverse_mass = [this, end_mass](int i) {
    return i == links() ? Scalar{1} / end_mass : Scalar{1};
  };
  const Scalar omega = Scalar{2} * Pi<Scalar>() * kJointHertz;
  Scalar sum{};
  for (int i = 1; i <= links(); ++i) {
    const Scalar load =
        kGravity * (static_cast<Scalar>(links() - i) + end_mass);
    const Scalar inverse_effective_mass =
        inverse_mass(i) + (i > 1 ? inverse_mass(i - 1) : Scalar{0});
    sum += load * inverse_effective_mass;
  }
  return sum / (omega * omega);
}

Scalar JointOrderSample::top_down_stretch() const {
  return Stretch(top_down_, links());
}

Scalar JointOrderSample::bottom_up_stretch() const {
  return Stretch(bottom_up_, links());
}

void JointOrderSample::Draw(DrawList& draw_list) const {
  DrawChain(top_down_, links(), sample_color::kWhite, draw_list);
  DrawChain(bottom_up_, links(), sample_color::kSky, draw_list);
}

void JointOrderSample::End() {
  top_down_.Clear();
  bottom_up_.Clear();
  for (auto& readout : readouts_) {
    readout.is_valid = false;
  }
}

}  // namespace neko::tutorial
