#include "tutorials/16_seesaw/solutions/over_constrained_sample.h"

#include "neko/math/angle.h"
#include "neko/math/const.h"
#include "neko/math/func.h"

#include "main/samples/body_draw.h"
#include "tutorials/16_seesaw/seesaw_sample.h"

namespace neko::tutorial {
namespace {
constexpr Vec2f kAnchorHalfSize{Scalar{4}, Ratio(1, 8)};
constexpr Vec2f kBalanceHalfSize{Scalar{2}, Ratio(1, 10)};
constexpr Vec2f kPlatformHalfSize{Scalar{1}, Ratio(3, 20)};
// The bodies by their centers from the axle: the anchor, the left and right
// balances, the left and right platforms
constexpr std::array<Vec2f, 5> kCenters{{
    {Scalar{0}, Scalar{0}},
    {-Scalar{4}, -Scalar{2}},
    {Scalar{4}, -Scalar{2}},
    {-Scalar{7}, -Ratio(27, 10)},
    {Scalar{7}, -Ratio(27, 10)},
}};
constexpr std::array<Vec2f, 5> kHalfSizes{{kAnchorHalfSize, kBalanceHalfSize,
                                           kBalanceHalfSize, kPlatformHalfSize,
                                           kPlatformHalfSize}};
// 1 / mass: the anchor 8 kg, the balances and the platforms 4 kg
constexpr std::array<Scalar, 5> kInverseMasses{
    {Ratio(1, 8), Ratio(1, 4), Ratio(1, 4), Ratio(1, 4), Ratio(1, 4)}};
constexpr Scalar kFriction = Ratio(2, 5);
constexpr Scalar kHingeRadius = Ratio(1, 10);
// The view: 1.5 times the world, centered on the rig
constexpr Scalar kViewScale = Ratio(3, 2);

int Millimeters(Scalar meters, int scale) {
  return ToDecimal(meters * Scalar{1000}, scale);
}
}  // namespace

void OverConstrainedSample::Begin(const SampleContext& context) {
  world_.emplace(
      PhysicsWorldConfig{.gravity = Vec2f{Scalar{0}, -Ratio(981, 100)},
                         .dt = kFixedDeltaTime,
                         .max_bodies = 6,
                         .max_colliders = 5,
                         .max_touching_pairs = 16,
                         .max_broadphase_pairs = 16,
                         .max_joints = 2,
                         .max_revolute_joints = 5,
                         .velocity_iterations = parameters_[kPasses].value});
  const Vec2f axle{context.world_bounds.GetCenter().x(),
                   context.world_bounds.max_bound().y() - Ratio(6, 5)};
  axle_ = AddAxle(*world_, axle);
  for (std::size_t i = 0; i < bodies_.size(); ++i) {
    bodies_[i] = AddPlank(*world_, axle + kCenters[i], kHalfSizes[i],
                          kInverseMasses[i], kFriction);
  }
  // The anchor on the axle, the balances on the anchor's ends, the
  // platforms on the balances' outer ends
  hinges_[0] = world_->AddRevoluteJoint(axle_, bodies_[0], axle);
  hinges_[1] = world_->AddRevoluteJoint(bodies_[0], bodies_[1],
                                        axle + Vec2f{-Scalar{4}, Scalar{0}});
  hinges_[2] = world_->AddRevoluteJoint(bodies_[0], bodies_[2],
                                        axle + Vec2f{Scalar{4}, Scalar{0}});
  hinges_[3] = world_->AddRevoluteJoint(bodies_[1], bodies_[3],
                                        axle + Vec2f{-Scalar{6}, -Scalar{2}});
  hinges_[4] = world_->AddRevoluteJoint(bodies_[2], bodies_[4],
                                        axle + Vec2f{Scalar{6}, -Scalar{2}});
  for (const RevoluteJointIndex hinge : hinges_) {
    SetSymmetricLimit(*world_, hinge, Radian{Degree{Scalar{45}}});
  }
  for (std::size_t i = 0; i < rods_.size(); ++i) {
    rods_[i] = world_->AddDistanceJoint(axle_, bodies_[3 + i]);
    world_->joint(rods_[i]).length = parameters_[kRodLength].ToScalar();
  }
}

void OverConstrainedSample::Measure() {
  for (const RevoluteJointIndex hinge : hinges_) {
    const auto anchors = world_->RevoluteJointAnchors(hinge);
    current_[0] = Max(current_[0], (anchors[1] - anchors[0]).Length());
  }
  for (const JointIndex rod : rods_) {
    current_[1] = Max(current_[1], Abs(world_->JointDistance(rod) -
                                       world_->joint(rod).length));
  }
  for (const BodyIndex body : bodies_) {
    current_[2] = Max(current_[2], world_->body(body).velocity.Length());
  }
}

void OverConstrainedSample::FixedUpdate() {
  world_->Step();
  ++step_;
  Measure();
  if (step_ % kWindowSteps == 0) {
    window_ = current_;
    current_ = {};
    readouts_[0].value = Millimeters(window_[0], readouts_[0].scale);
    readouts_[1].value = Millimeters(window_[1], readouts_[1].scale);
    readouts_[2].value = ToDecimal(window_[2], readouts_[2].scale);
    for (std::size_t i = 0; i < 3; ++i) {
      readouts_[i].is_valid = true;
    }
  }
  readouts_[3].value = Millimeters(anchor_drift(), readouts_[3].scale);
  readouts_[3].is_valid = true;
}

void OverConstrainedSample::Draw(DrawList& draw_list) const {
  const Vec2f view_center =
      world_->body(axle_).position - Vec2f{Scalar{0}, Ratio(12, 5)};
  const Vec2f view_half =
      (draw_list.camera.max_bound() - draw_list.camera.min_bound()) *
      (kViewScale / Scalar{2});
  draw_list.camera = Aabbf{view_center - view_half, view_center + view_half};
  for (std::size_t i = 0; i < rods_.size(); ++i) {
    const auto anchors = world_->JointAnchors(rods_[i]);
    const bool is_short = world_->JointDistance(rods_[i]) <
                          world_->joint(rods_[i]).length - Ratio(1, 100);
    draw_list.lines.push_back(
        {anchors[0], anchors[1],
         is_short ? sample_color::kRed : sample_color::kOrange});
  }
  for (std::size_t i = 0; i < bodies_.size(); ++i) {
    const Body& body = world_->body(bodies_[i]);
    DrawBoxOutline(body.position, kHalfSizes[i], body.rotation,
                   i < 3 ? sample_color::kWhite : sample_color::kSky,
                   draw_list);
  }
  for (const RevoluteJointIndex hinge : hinges_) {
    draw_list.circles.push_back({world_->RevoluteJointAnchors(hinge)[0],
                                 kHingeRadius, sample_color::kGrey});
  }
}

void OverConstrainedSample::End() {
  world_.reset();
  step_ = 0;
  window_ = {};
  current_ = {};
  for (auto& readout : readouts_) {
    readout.is_valid = false;
  }
}

Scalar OverConstrainedSample::anchor_drift() const {
  return (world_->body(bodies_[0]).position - world_->body(axle_).position)
      .Length();
}

}  // namespace neko::tutorial
