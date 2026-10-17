#include "tutorials/09_spring/solutions/substeps_sample.h"

#include "neko/math/const.h"
#include "neko/math/func.h"

namespace neko::tutorial {
namespace {
constexpr Scalar kGravity = Ratio(981, 100);
constexpr Scalar kRestLength{2};
constexpr Scalar kMassRadius = Ratio(1, 5);
constexpr Scalar kAnchorRadius = Ratio(1, 20);
constexpr Scalar kCoilHalfWidth = Ratio(3, 20);
constexpr int kCoils = 12;
// Past this it blew up: stop stepping. Fixed16 limit: k x / m must stay
// under 32767 m/s^2 (1.6 m for 2000 N/m on 0.1 kg)
constexpr Scalar kExplodedDistance = Ratio(3, 2);
}  // namespace

void SubstepsSample::Begin(const SampleContext& context) {
  const int substeps = parameters_[kSubsteps].value;
  world_ = PhysicsWorld{{.gravity = Vec2f{Scalar{0}, -kGravity},
                         .dt = kFixedDeltaTime / static_cast<Scalar>(substeps),
                         .max_bodies = 2,
                         .max_colliders = 1,
                         .max_touching_pairs = 1,
                         .max_broadphase_pairs = 1,
                         .max_joints = 1}};
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
  step_ = 0;
  crossings_ = 0;
  exploded_ = false;
  previous_displacement_ = displacement();

  readouts_[0].value = ToDecimal(omega_dt(), readouts_[0].scale);
  readouts_[1].value = ToDecimal(predicted_period(), readouts_[1].scale);
  readouts_[3].value = 0;
  for (const std::size_t i : {0u, 1u, 3u}) {
    readouts_[i].is_valid = true;
  }
}

void SubstepsSample::FixedUpdate() {
  for (int k = 0; k < parameters_[kSubsteps].value && !exploded_; ++k) {
    world_.Step();
    ++step_;
    const Scalar now = displacement();
    if (Abs(now) > kExplodedDistance) {
      exploded_ = true;
      readouts_[3].value = 1;
      return;
    }
    if ((previous_displacement_ < Scalar{0}) != (now < Scalar{0})) {
      const Scalar fraction =
          previous_displacement_ / (previous_displacement_ - now);
      const Scalar time =
          (static_cast<Scalar>(step_ - 1) + fraction) * world_.dt();
      if (crossings_ == 0) {
        first_crossing_ = time;
      }
      last_crossing_ = time;
      ++crossings_;
    }
    previous_displacement_ = now;
  }
  if (const auto period = measured_period()) {
    readouts_[2].value = ToDecimal(*period, readouts_[2].scale);
    readouts_[2].is_valid = true;
  }
}

Scalar SubstepsSample::predicted_period() const {
  // Not Sqrt(m / k): 0.1 / 2000 is 3 units of 1/65536 in Fixed16
  return Scalar{2} * Pi<Scalar>() * Sqrt(mass()) / Sqrt(stiffness());
}

std::optional<Scalar> SubstepsSample::measured_period() const {
  if (crossings_ < 3) {
    return std::nullopt;
  }
  return Scalar{2} * (last_crossing_ - first_crossing_) /
         static_cast<Scalar>(crossings_ - 1);
}

Scalar SubstepsSample::omega_dt() const {
  return Sqrt(stiffness() / mass()) * world_.dt();
}

Scalar SubstepsSample::displacement() const {
  return equilibrium_y_ - world_.body(mass_).position.y();
}

void SubstepsSample::Draw(DrawList& draw_list) const {
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
  draw_list.circles.push_back({top, kAnchorRadius, sample_color::kGrey});
  draw_list.circles.push_back({bottom, kMassRadius, sample_color::kOrange});
}

void SubstepsSample::End() {
  world_.Clear();
  step_ = 0;
  crossings_ = 0;
  exploded_ = false;
  for (auto& readout : readouts_) {
    readout.is_valid = false;
  }
}

}  // namespace neko::tutorial
