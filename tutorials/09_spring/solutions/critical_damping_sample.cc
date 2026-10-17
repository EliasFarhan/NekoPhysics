#include "tutorials/09_spring/solutions/critical_damping_sample.h"

#include "neko/math/func.h"

namespace neko::tutorial {
namespace {
constexpr Scalar kGravity = Ratio(981, 100);
constexpr Scalar kRestLength{2};
constexpr Scalar kMassRadius = Ratio(1, 5);
constexpr Scalar kAnchorRadius = Ratio(1, 20);
constexpr Scalar kCoilHalfWidth = Ratio(3, 20);
constexpr int kCoils = 12;
}  // namespace

void CriticalDampingSample::Begin(const SampleContext& context) {
  const Vec2f top{context.world_bounds.GetCenter().x(),
                  context.world_bounds.max_bound().y() - Ratio(1, 2)};
  anchor_ = world_.AddBody();
  world_.body(anchor_).type = BodyType::kStatic;
  world_.body(anchor_).inverse_mass = Scalar{0};
  world_.body(anchor_).position = top;

  equilibrium_y_ = top.y() - kRestLength - mass() * kGravity / stiffness();
  mass_ = world_.AddBody();
  world_.body(mass_).inverse_mass = Scalar{1} / mass();
  world_.body(mass_).position = {top.x(), equilibrium_y_ - kRelease};

  const JointIndex spring_index = world_.AddDistanceJoint(anchor_, mass_);
  DistanceJoint& spring = world_.joint(spring_index);
  spring.type = DistanceJointType::kSpring;
  spring.length = kRestLength;
  spring.stiffness = stiffness();
  spring.damping = damping();
  step_ = 0;
  settling_time_ = Scalar{0};
  overshoot_ = Scalar{0};

  readouts_[0].value = ToDecimal(damping(), readouts_[0].scale);
  for (auto& readout : readouts_) {
    readout.is_valid = true;
  }
  readouts_[1].value = 0;
  readouts_[2].value = 0;
}

void CriticalDampingSample::FixedUpdate() {
  world_.Step();
  ++step_;
  const Scalar now = displacement();
  if (Abs(now) > kBand) {
    settling_time_ = static_cast<Scalar>(step_) * world_.dt();
  }
  overshoot_ = Max(overshoot_, -now);
  readouts_[1].value = ToDecimal(settling_time_, readouts_[1].scale);
  readouts_[2].value = ToDecimal(overshoot_ * Scalar{1000}, readouts_[2].scale);
}

Scalar CriticalDampingSample::damping() const {
  return Scalar{2} * parameters_[kDampingRatio].ToScalar() *
         Sqrt(stiffness() * mass());
}

Scalar CriticalDampingSample::displacement() const {
  return equilibrium_y_ - world_.body(mass_).position.y();
}

void CriticalDampingSample::Draw(DrawList& draw_list) const {
  const Vec2f top = world_.body(anchor_).position;
  const Vec2f bottom = world_.body(mass_).position;
  const Vec2f along = (bottom - top) / static_cast<Scalar>(kCoils + 1);
  const Vec2f side = Vec2f::Right() * kCoilHalfWidth;
  Vec2f previous = top;
  for (int i = 1; i <= kCoils; ++i) {
    const Vec2f corner =
        top + along * static_cast<Scalar>(i) + (i % 2 == 0 ? side : -side);
    draw_list.lines.push_back({previous, corner, sample_color::kWhite});
    previous = corner;
  }
  draw_list.lines.push_back({previous, bottom, sample_color::kWhite});
  draw_list.lines.push_back({Vec2f{top.x() - Scalar{1}, equilibrium_y_},
                             Vec2f{top.x() - Ratio(1, 2), equilibrium_y_},
                             sample_color::kGreen});
  for (const Scalar edge : {equilibrium_y_ - kBand, equilibrium_y_ + kBand}) {
    draw_list.lines.push_back({Vec2f{top.x() + Ratio(1, 2), edge},
                               Vec2f{top.x() + Scalar{1}, edge},
                               sample_color::kGrey});
  }
  draw_list.circles.push_back({top, kAnchorRadius, sample_color::kGrey});
  draw_list.circles.push_back({bottom, kMassRadius, sample_color::kOrange});
}

void CriticalDampingSample::End() {
  world_.Clear();
  step_ = 0;
  for (auto& readout : readouts_) {
    readout.is_valid = false;
  }
}

}  // namespace neko::tutorial
