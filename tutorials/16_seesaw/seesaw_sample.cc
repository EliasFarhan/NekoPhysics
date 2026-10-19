#include "tutorials/16_seesaw/seesaw_sample.h"

#include <array>

#include "neko/math/angle.h"
#include "neko/math/const.h"
#include "neko/math/func.h"

#include "main/samples/body_draw.h"

namespace neko::tutorial {
namespace {
constexpr Scalar kGravity = Ratio(981, 100);
// The plank: 8 m long, 8 kg, its axle 3 m under the top of the world
constexpr Vec2f kPlankHalfSize{Scalar{4}, Ratio(1, 8)};
constexpr Scalar kPlankMass{8};
constexpr Scalar kAxleDepth = Ratio(13, 5);
// The pans: 1.8 m wide with 0.3 m lips, 4 kg, 1.6 m under the plank's ends
constexpr Vec2f kPanHalfSize{Ratio(9, 10), Ratio(1, 10)};
constexpr Vec2f kLipHalfSize{Ratio(1, 20), Ratio(3, 20)};
constexpr Scalar kPanMass{4};
constexpr Scalar kPanDrop = Ratio(8, 5);
constexpr Scalar kFriction = Ratio(2, 5);
// The crates: 0.4 m, 1 kg
constexpr Vec2f kCrateHalfSize{Ratio(1, 5), Ratio(1, 5)};
constexpr Scalar kCrateMass{1};
// Where the crates appear in a pan, in its frame: three on its floor, two
// on them, 5 cm above their place
constexpr std::array<Vec2f, 5> kCrateSlots{{
    {-Ratio(3, 5), Ratio(7, 20)},
    {Ratio(3, 5), Ratio(7, 20)},
    {Scalar{0}, Ratio(7, 20)},
    {-Ratio(3, 10), Ratio(4, 5)},
    {Ratio(3, 10), Ratio(4, 5)},
}};
constexpr int kFirstDrop = 20;
constexpr int kSettleSteps = 600;
constexpr Scalar kHingeRadius = Ratio(1, 10);

int Millimeters(Scalar meters, int scale) {
  return ToDecimal(meters * Scalar{1000}, scale);
}
}  // namespace

BodyIndex AddAxle(PhysicsWorld& world, Vec2f position) {
  const BodyIndex body = world.AddBody();
  world.body(body).type = BodyType::kStatic;
  world.body(body).inverse_mass = Scalar{0};
  world.body(body).position = position;
  return body;
}

BodyIndex AddPlank(PhysicsWorld& world, Vec2f center, Vec2f half_size,
                   Scalar inverse_mass, Scalar friction) {
  const BodyIndex body = world.AddBody();
  world.body(body).position = center;
  world.body(body).inverse_mass = inverse_mass;
  const ColliderIndex index = world.AddOrientedBoxCollider(body);
  Collider& collider = world.collider(index);
  collider.is_trigger = false;
  collider.restitution = Scalar{0};
  collider.friction = friction;
  world.oriented_box(collider.shape_index).half_size = half_size;
  return body;
}

void SetSymmetricLimit(PhysicsWorld& world, RevoluteJointIndex joint,
                       Radian limit) {
  RevoluteJoint& data = world.revolute_joint(joint);
  data.enable_limit = true;
  data.lower_angle = -limit;
  data.upper_angle = limit;
}

void SeesawSample::Begin(const SampleContext& context) {
  const Vec2f axle_position{context.world_bounds.GetCenter().x(),
                            context.world_bounds.max_bound().y() - kAxleDepth};
  axle_ = AddAxle(world_, axle_position);
  plank_ = AddPlank(world_, axle_position, kPlankHalfSize,
                    Scalar{1} / kPlankMass, kFriction);
  axle_joint_ = world_.AddRevoluteJoint(axle_, plank_, axle_position);
  SetSymmetricLimit(
      world_, axle_joint_,
      Radian{Degree{static_cast<Scalar>(parameters_[kLimit].value)}});

  for (std::size_t side = 0; side < 2; ++side) {
    const Scalar sign = side == 0 ? Scalar{-1} : Scalar{1};
    const Vec2f hinge =
        axle_position + Vec2f{sign * kPlankHalfSize.x(), Scalar{0}};
    const BodyIndex pan =
        AddPlank(world_, hinge - Vec2f{Scalar{0}, kPanDrop}, kPanHalfSize,
                 Scalar{1} / kPanMass, kFriction);
    // Two lips keep the crates in when the pan swings
    for (const Scalar lip_side : {Scalar{-1}, Scalar{1}}) {
      const ColliderIndex index = world_.AddOrientedBoxCollider(pan);
      Collider& lip = world_.collider(index);
      lip.is_trigger = false;
      lip.restitution = Scalar{0};
      lip.friction = kFriction;
      lip.offset = {lip_side * (kPanHalfSize.x() - kLipHalfSize.x()),
                    kPanHalfSize.y() + kLipHalfSize.y()};
      world_.oriented_box(lip.shape_index).half_size = kLipHalfSize;
    }
    pans_[side] = pan;
    pan_joints_[side] = world_.AddRevoluteJoint(plank_, pan, hinge);
    SetSymmetricLimit(world_, pan_joints_[side], Radian{Degree{Scalar{45}}});
  }
  readouts_[4].value = ToDecimal(weight(), readouts_[4].scale);
  readouts_[4].is_valid = true;
}

void SeesawSample::DropCrate(int side) {
  const auto index = static_cast<std::size_t>(side);
  // Loaded straight into the pan (a crate dropped from above would land on
  // the plank's end, right over the pan), turned with it
  const Body& pan = world_.body(pans_[index]);
  const Vec2f slot = kCrateSlots[static_cast<std::size_t>(dropped_[index])];
  const BodyIndex crate =
      AddPlank(world_, pan.position + pan.rotation.Rotate(slot), kCrateHalfSize,
               Scalar{1} / kCrateMass, kFriction);
  world_.body(crate).rotation = pan.rotation;
  world_.body(crate).velocity = pan.velocity;
  crates_[static_cast<std::size_t>(crate_count_)] = crate;
  ++crate_count_;
  ++dropped_[index];
}

void SeesawSample::DampHinge(RevoluteJointIndex joint) {
  const RevoluteJoint& data = world_.revolute_joint(joint);
  Body& body_a = world_.body(data.body_a);
  Body& body_b = world_.body(data.body_b);
  const Scalar torque = -parameters_[kDamping].ToScalar() *
                        (body_b.angular_velocity - body_a.angular_velocity);
  // Equal and opposite; a static body ignores its torque
  body_a.torque -= torque;
  body_b.torque += torque;
}

void SeesawSample::FixedUpdate() {
  // One crate every kDropPeriod steps, left and right in turn, until each
  // side has its count
  const std::array<int, 2> wanted{parameters_[kLeft].value,
                                  parameters_[kRight].value};
  if (step_ >= kFirstDrop && (step_ - kFirstDrop) % kDropPeriod == 0 &&
      crate_count_ < wanted[0] + wanted[1]) {
    int side = next_side_;
    if (dropped_[static_cast<std::size_t>(side)] >=
        wanted[static_cast<std::size_t>(side)]) {
      side = 1 - side;
    }
    DropCrate(side);
    next_side_ = 1 - side;
  }
  for (const RevoluteJointIndex joint :
       {axle_joint_, pan_joints_[0], pan_joints_[1]}) {
    DampHinge(joint);
  }
  world_.Step();
  ++step_;
  for (const RevoluteJointIndex joint :
       {axle_joint_, pan_joints_[0], pan_joints_[1]}) {
    const auto anchors = world_.RevoluteJointAnchors(joint);
    max_pin_gap_ = Max(max_pin_gap_, (anchors[1] - anchors[0]).Length());
  }

  readouts_[0].value =
      ToDecimal(Degree{plank_angle()}.value(), readouts_[0].scale);
  readouts_[1].value = ToDecimal(predicted_torque(), readouts_[1].scale);
  readouts_[2].value = ToDecimal(limit_torque(), readouts_[2].scale);
  readouts_[3].value = ToDecimal(axle_force(), readouts_[3].scale);
  readouts_[4].value = ToDecimal(weight(), readouts_[4].scale);
  readouts_[5].value = Millimeters(max_pin_gap_, readouts_[5].scale);
  for (auto& readout : readouts_) {
    readout.is_valid = true;
  }
}

void SeesawSample::Draw(DrawList& draw_list) const {
  const Body& axle = world_.body(axle_);
  const Body& plank = world_.body(plank_);
  // The limits, from the axle along the plank's two extreme directions
  const Radian limit = world_.revolute_joint(axle_joint_).upper_angle;
  for (const Radian angle : {limit, -limit}) {
    const Vec2f direction{Cos(angle), Sin(angle)};
    const Vec2f reach = direction * kPlankHalfSize.x();
    draw_list.lines.push_back({axle.position - reach, axle.position + reach,
                               sample_color::kDarkGrey});
  }
  DrawBoxOutline(plank.position, kPlankHalfSize, plank.rotation,
                 sample_color::kWhite, draw_list);
  for (std::size_t side = 0; side < 2; ++side) {
    const Body& pan = world_.body(pans_[side]);
    const auto anchors = world_.RevoluteJointAnchors(pan_joints_[side]);
    draw_list.lines.push_back({anchors[1], pan.position, sample_color::kSky});
    DrawBoxOutline(pan.position, kPanHalfSize, pan.rotation, sample_color::kSky,
                   draw_list);
    for (const Scalar lip_side : {Scalar{-1}, Scalar{1}}) {
      const Vec2f offset{lip_side * (kPanHalfSize.x() - kLipHalfSize.x()),
                         kPanHalfSize.y() + kLipHalfSize.y()};
      DrawBoxOutline(pan.position + pan.rotation.Rotate(offset), kLipHalfSize,
                     pan.rotation, sample_color::kSky, draw_list);
    }
    draw_list.circles.push_back(
        {anchors[0], kHingeRadius, sample_color::kGrey});
  }
  draw_list.circles.push_back(
      {axle.position, kHingeRadius, sample_color::kGrey});
  for (int i = 0; i < crate_count_; ++i) {
    const Body& crate = world_.body(crates_[static_cast<std::size_t>(i)]);
    DrawBoxOutline(crate.position, kCrateHalfSize, crate.rotation,
                   sample_color::kOrange, draw_list);
  }
}

void SeesawSample::End() {
  world_.Clear();
  crate_count_ = 0;
  dropped_ = {};
  next_side_ = 0;
  step_ = 0;
  max_pin_gap_ = Scalar{0};
  for (auto& readout : readouts_) {
    readout.is_valid = false;
  }
}

Scalar SeesawSample::predicted_torque() const {
  const Scalar mass_difference =
      static_cast<Scalar>(dropped_[0] - dropped_[1]) * kCrateMass;
  return kGravity * mass_difference * kPlankHalfSize.x() * Cos(plank_angle());
}

Scalar SeesawSample::limit_torque() const {
  const RevoluteJoint& joint = world_.revolute_joint(axle_joint_);
  return (joint.upper_impulse - joint.lower_impulse) / world_.dt();
}

Scalar SeesawSample::axle_force() const {
  return world_.revolute_joint(axle_joint_).impulse.Length() / world_.dt();
}

Scalar SeesawSample::weight() const {
  return kGravity * (kPlankMass + Scalar{2} * kPanMass +
                     static_cast<Scalar>(crate_count_) * kCrateMass);
}

int SeesawSample::settled_steps() const {
  return kFirstDrop +
         kDropPeriod * (parameters_[kLeft].value + parameters_[kRight].value) +
         kSettleSteps;
}

}  // namespace neko::tutorial
