#include "tutorials/09_spring/solutions/stability_limit_sample.h"

#include "neko/math/func.h"

namespace neko::tutorial {
namespace {
constexpr Scalar kGravity = Ratio(981, 100);
constexpr Scalar kRestLength{2};
constexpr Scalar kMassRadius = Ratio(1, 5);
constexpr Scalar kAnchorRadius = Ratio(1, 20);
// 10 s
constexpr int kTrialSteps = 600;
// Anchor at body 0, mass at body 1, the spring at joint 0
constexpr BodyIndex kAnchor{0};
constexpr BodyIndex kMassBody{1};

// A mass on a spring of stiffness k, released kRelease below its
// equilibrium; returns the equilibrium height
Scalar BuildSpring(PhysicsWorld& world, Vec2f top, Scalar mass,
                   Scalar stiffness) {
  world = PhysicsWorld{{.gravity = Vec2f{Scalar{0}, -kGravity},
                        .dt = Sample::kFixedDeltaTime,
                        .max_bodies = 2,
                        .max_colliders = 1,
                        .max_touching_pairs = 1,
                        .max_broadphase_pairs = 1,
                        .max_joints = 1}};
  const BodyIndex anchor = world.AddBody();
  world.body(anchor).type = BodyType::kStatic;
  world.body(anchor).inverse_mass = Scalar{0};
  world.body(anchor).position = top;
  const Scalar equilibrium_y =
      top.y() - kRestLength - mass * kGravity / stiffness;
  const BodyIndex body = world.AddBody();
  world.body(body).inverse_mass = Scalar{1} / mass;
  world.body(body).position = {top.x(),
                               equilibrium_y - StabilityLimitSample::kRelease};
  const JointIndex spring_index = world.AddDistanceJoint(anchor, body);
  DistanceJoint& spring = world.joint(spring_index);
  spring.type = DistanceJointType::kSpring;
  spring.length = kRestLength;
  spring.stiffness = stiffness;
  return equilibrium_y;
}

void DrawSpring(const PhysicsWorld& world, Scalar equilibrium_y,
                DrawList& draw_list) {
  const Vec2f top = world.body(kAnchor).position;
  const Vec2f bottom = world.body(kMassBody).position;
  draw_list.lines.push_back({top, bottom, sample_color::kWhite});
  draw_list.lines.push_back({Vec2f{top.x() - Ratio(1, 2), equilibrium_y},
                             Vec2f{top.x() - Ratio(1, 4), equilibrium_y},
                             sample_color::kGreen});
  draw_list.circles.push_back({top, kAnchorRadius, sample_color::kGrey});
  draw_list.circles.push_back({bottom, kMassRadius, sample_color::kOrange});
}
}  // namespace

bool StabilityLimitSample::IsStable(Scalar stiffness) const {
  PhysicsWorld world{{.max_bodies = 1, .max_colliders = 1, .max_joints = 1}};
  const Scalar equilibrium_y =
      BuildSpring(world, Vec2f{Scalar{0}, Scalar{5}}, mass(), stiffness);
  for (int step = 0; step < kTrialSteps; ++step) {
    world.Step();
    if (Abs(world.body(kMassBody).position.y() - equilibrium_y) > kBound) {
      return false;
    }
  }
  return true;
}

void StabilityLimitSample::Begin(const SampleContext& context) {
  // Bisection between half and twice the prediction, to 1 N/m
  Scalar stable = predicted_limit() / Scalar{2};
  Scalar unstable = predicted_limit() * Scalar{2};
  while (unstable - stable > Scalar{1}) {
    // Not (stable + unstable) / 2: the sum passes Fixed16's 32767 for 1 kg
    const Scalar middle = stable + (unstable - stable) / Scalar{2};
    if (IsStable(middle)) {
      stable = middle;
    } else {
      unstable = middle;
    }
  }
  measured_limit_ = stable;

  const Vec2f center = context.world_bounds.GetCenter();
  const Scalar top = context.world_bounds.max_bound().y() - Scalar{1};
  below_equilibrium_y_ = BuildSpring(below_, Vec2f{center.x() - Scalar{2}, top},
                                     mass(), measured_limit_ * Ratio(95, 100));
  above_equilibrium_y_ = BuildSpring(above_, Vec2f{center.x() + Scalar{2}, top},
                                     mass(), measured_limit_ * Ratio(105, 100));
  above_exploded_ = false;

  readouts_[0].value = ToDecimal(predicted_limit(), readouts_[0].scale);
  readouts_[1].value = ToDecimal(measured_limit_, readouts_[1].scale);
  readouts_[2].value = ToDecimal(Sqrt(measured_limit_ / mass()) * below_.dt(),
                                 readouts_[2].scale);
  for (auto& readout : readouts_) {
    readout.is_valid = true;
  }
}

void StabilityLimitSample::FixedUpdate() {
  below_.Step();
  if (!above_exploded_) {
    above_.Step();
    above_exploded_ = Abs(above_displacement()) > kBound;
  }
}

Scalar StabilityLimitSample::predicted_limit() const {
  // dt^2 is 18 units of 1/65536 in Fixed16: square 1/dt instead
  const Scalar rate = Scalar{1} / kFixedDeltaTime;
  return Scalar{4} * mass() * rate * rate;
}

Scalar StabilityLimitSample::above_displacement() const {
  return above_equilibrium_y_ - above_.body(kMassBody).position.y();
}

void StabilityLimitSample::Draw(DrawList& draw_list) const {
  DrawSpring(below_, below_equilibrium_y_, draw_list);
  DrawSpring(above_, above_equilibrium_y_, draw_list);
}

void StabilityLimitSample::End() {
  below_.Clear();
  above_.Clear();
  for (auto& readout : readouts_) {
    readout.is_valid = false;
  }
}

}  // namespace neko::tutorial
