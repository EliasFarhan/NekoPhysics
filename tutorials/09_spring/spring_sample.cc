#include "tutorials/09_spring/spring_sample.h"

#include "neko/math/const.h"
#include "neko/math/func.h"

namespace neko::tutorial {
namespace {
constexpr Scalar kGravity = Ratio(981, 100);
constexpr Scalar kRestLength{2};
constexpr Scalar kMassRadius = Ratio(1, 5);
constexpr Scalar kAnchorRadius = Ratio(1, 20);
// Half width of the zigzag and its number of corners
constexpr Scalar kCoilHalfWidth = Ratio(3, 20);
constexpr int kCoils = 12;
// Past this the spring blew up: stop stepping
constexpr Scalar kExplodedDistance{100};
}  // namespace

void SpringSample::Begin(const SampleContext& context) {
  const Vec2f top{context.world_bounds.GetCenter().x(),
                  context.world_bounds.max_bound().y() - Ratio(1, 2)};
  anchor_ = world_.AddBody();
  world_.body(anchor_).type = BodyType::kStatic;
  world_.body(anchor_).inverse_mass = Scalar{0};
  world_.body(anchor_).position = top;

  equilibrium_y_ = top.y() - kRestLength - equilibrium_stretch();
  mass_ = world_.AddBody();
  world_.body(mass_).inverse_mass = Scalar{1} / mass();
  world_.body(mass_).position = {top.x(), equilibrium_y_ - kRelease};

  spring_ = world_.AddDistanceJoint(anchor_, mass_);
  DistanceJoint& spring = world_.joint(spring_);
  spring.type = DistanceJointType::kSpring;
  spring.length = kRestLength;
  spring.stiffness = stiffness();
  spring.damping = parameters_[kDamping].ToScalar();
  previous_displacement_ = displacement();

  readouts_[0].value = ToDecimal(predicted_period(), readouts_[0].scale);
  readouts_[2].value = ToDecimal(equilibrium_stretch(), readouts_[2].scale);
  readouts_[3].value = ToDecimal(omega_dt(), readouts_[3].scale);
  readouts_[4].value = ToDecimal(damping_ratio(), readouts_[4].scale);
  for (const std::size_t i : {0u, 2u, 3u, 4u, 5u}) {
    readouts_[i].is_valid = true;
  }
  readouts_[5].value = 0;
}

void SpringSample::FixedUpdate() {
  if (exploded_) {
    return;
  }
  world_.Step();
  ++step_;
  const Scalar now = displacement();
  if (Abs(now) > kExplodedDistance) {
    exploded_ = true;
    readouts_[5].value = 1;
    return;
  }
  // Crossing the equilibrium: interpolate the time between the two steps
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
  if (const auto period = measured_period()) {
    readouts_[1].value = ToDecimal(*period, readouts_[1].scale);
    readouts_[1].is_valid = true;
  }
}

void SpringSample::Draw(DrawList& draw_list) const {
  const Vec2f top = world_.body(anchor_).position;
  const Vec2f bottom = world_.body(mass_).position;
  // Zigzag: kCoils corners alternating left and right of the axis
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

void SpringSample::End() {
  world_.Clear();
  step_ = 0;
  crossings_ = 0;
  first_crossing_ = Scalar{0};
  last_crossing_ = Scalar{0};
  exploded_ = false;
  for (auto& readout : readouts_) {
    readout.is_valid = false;
  }
}

Scalar SpringSample::predicted_period() const {
  return Scalar{2} * Pi<Scalar>() * Sqrt(mass() / stiffness());
}

std::optional<Scalar> SpringSample::measured_period() const {
  if (crossings_ < 3) {
    return std::nullopt;
  }
  return Scalar{2} * (last_crossing_ - first_crossing_) /
         static_cast<Scalar>(crossings_ - 1);
}

Scalar SpringSample::equilibrium_stretch() const {
  return mass() * kGravity / stiffness();
}

Scalar SpringSample::omega_dt() const {
  return Sqrt(stiffness() / mass()) * kFixedDeltaTime;
}

Scalar SpringSample::damping_ratio() const {
  return parameters_[kDamping].ToScalar() /
         (Scalar{2} * Sqrt(stiffness() * mass()));
}

Scalar SpringSample::displacement() const {
  return equilibrium_y_ - world_.body(mass_).position.y();
}

}  // namespace neko::tutorial
