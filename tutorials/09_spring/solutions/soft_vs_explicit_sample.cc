#include "tutorials/09_spring/solutions/soft_vs_explicit_sample.h"

#include "neko/math/const.h"
#include "neko/math/func.h"

namespace neko::tutorial {
namespace {
constexpr Scalar kGravity = Ratio(981, 100);
constexpr Scalar kLength{2};
constexpr Scalar kBobRadius = Ratio(3, 20);
constexpr Scalar kAnchorRadius = Ratio(1, 20);
// Past this the spring blew up: stop stepping
constexpr Scalar kExplodedStretch{1};
// Bodies: rod anchor, rod bob, spring anchor, spring bob
constexpr BodyIndex kRodAnchor{0};
constexpr BodyIndex kRodBob{1};
constexpr BodyIndex kSpringAnchor{2};
constexpr BodyIndex kSpringBob{3};

void AddHanging(PhysicsWorld& world, Vec2f top) {
  const BodyIndex anchor = world.AddBody();
  world.body(anchor).type = BodyType::kStatic;
  world.body(anchor).inverse_mass = Scalar{0};
  world.body(anchor).position = top;
  const BodyIndex bob = world.AddBody();
  world.body(bob).inverse_mass = Scalar{1};
  world.body(bob).position = top + Vec2f{Scalar{0}, -kLength};
}
}  // namespace

void SoftVsExplicitSample::Begin(const SampleContext& context) {
  world_ = PhysicsWorld{
      {.gravity = Vec2f{Scalar{0}, -kGravity},
       .dt = kFixedDeltaTime,
       .max_bodies = 4,
       .max_colliders = 1,
       .max_touching_pairs = 1,
       .max_broadphase_pairs = 1,
       .max_joints = 2,
       .joint_hertz = static_cast<Scalar>(parameters_[kHertz].value)}};
  const Vec2f center = context.world_bounds.GetCenter();
  const Scalar top = context.world_bounds.max_bound().y() - Scalar{2};
  AddHanging(world_, {center.x() - Scalar{1}, top});
  AddHanging(world_, {center.x() + Scalar{1}, top});
  rod_ = world_.AddDistanceJoint(kRodAnchor, kRodBob);
  spring_ = world_.AddDistanceJoint(kSpringAnchor, kSpringBob);
  DistanceJoint& spring = world_.joint(spring_);
  spring.type = DistanceJointType::kSpring;
  // A 1 kg bob: k = w^2, c = 2 zeta w
  spring.stiffness = omega() * omega();
  spring.damping = Scalar{2} * parameters_[kDampingRatio].ToScalar() * omega();
  spring_ringing_ = Scalar{0};
  exploded_ = false;

  readouts_[0].value =
      ToDecimal(predicted_sag() * Scalar{1000}, readouts_[0].scale);
  readouts_[3].value = ToDecimal(spring.stiffness, readouts_[3].scale);
  readouts_[4].value = ToDecimal(spring.damping, readouts_[4].scale);
  readouts_[5].value = ToDecimal(stability_number(), readouts_[5].scale);
  for (auto& readout : readouts_) {
    readout.is_valid = true;
  }
  readouts_[1].value = 0;
  readouts_[2].value = 0;
  readouts_[6].value = 0;
}

void SoftVsExplicitSample::FixedUpdate() {
  if (exploded_) {
    return;
  }
  world_.Step();
  if (Abs(spring_stretch()) > kExplodedStretch) {
    exploded_ = true;
    readouts_[6].value = 1;
    return;
  }
  spring_ringing_ =
      Max(spring_ringing_, Abs(spring_stretch() - predicted_sag()));
  readouts_[1].value =
      ToDecimal(rod_stretch() * Scalar{1000}, readouts_[1].scale);
  readouts_[2].value =
      ToDecimal(spring_stretch() * Scalar{1000}, readouts_[2].scale);
}

Scalar SoftVsExplicitSample::omega() const {
  return Scalar{2} * Pi<Scalar>() *
         static_cast<Scalar>(parameters_[kHertz].value);
}

Scalar SoftVsExplicitSample::predicted_sag() const {
  return kGravity / (omega() * omega());
}

Scalar SoftVsExplicitSample::rod_stretch() const {
  return world_.JointDistance(rod_) - kLength;
}

Scalar SoftVsExplicitSample::spring_stretch() const {
  return world_.JointDistance(spring_) - kLength;
}

Scalar SoftVsExplicitSample::stability_number() const {
  const Scalar omega_dt = omega() * world_.dt();
  return omega_dt * omega_dt +
         Scalar{4} * parameters_[kDampingRatio].ToScalar() * omega_dt;
}

void SoftVsExplicitSample::Draw(DrawList& draw_list) const {
  draw_list.lines.push_back({world_.body(kRodAnchor).position,
                             world_.body(kRodBob).position,
                             sample_color::kWhite});
  draw_list.lines.push_back({world_.body(kSpringAnchor).position,
                             world_.body(kSpringBob).position,
                             sample_color::kSky});
  for (const BodyIndex anchor : {kRodAnchor, kSpringAnchor}) {
    draw_list.circles.push_back(
        {world_.body(anchor).position, kAnchorRadius, sample_color::kGrey});
  }
  for (const BodyIndex bob : {kRodBob, kSpringBob}) {
    draw_list.circles.push_back(
        {world_.body(bob).position, kBobRadius, sample_color::kOrange});
  }
}

void SoftVsExplicitSample::End() {
  world_.Clear();
  for (auto& readout : readouts_) {
    readout.is_valid = false;
  }
}

}  // namespace neko::tutorial
