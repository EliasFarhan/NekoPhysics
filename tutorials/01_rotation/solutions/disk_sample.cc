#include "tutorials/01_rotation/solutions/disk_sample.h"

#include "neko/math/func.h"

#include "main/samples/body_draw.h"

namespace neko::tutorial {
namespace {
// The box of chapter 01, to compare with
constexpr Scalar kBoxWidth = Scalar{2};
constexpr Scalar kBoxHeight = Scalar{1};
// Forces drawn as arrows of this many meters per newton
constexpr Scalar kForceDrawScale = Ratio(1, 4);
// Steps per second of the push time parameter (dt = 1/60 s)
constexpr int kStepsPerSecond = 60;
}  // namespace

void DiskSample::Begin(const SampleContext& context) {
  const SampleParameter& push_time = parameters_[kPushTime];
  push_steps_ = push_time.value * kStepsPerSecond / push_time.scale;

  disk_index_ = world_.AddBody();
  Body& disk = world_.body(disk_index_);
  disk.position = context.world_bounds.GetCenter();
  disk.inverse_mass = Scalar{1} / parameters_[kMass].ToScalar();
  const ColliderIndex collider_index = world_.AddCircleCollider(disk_index_);
  Collider& collider = world_.collider(collider_index);
  collider.is_trigger = false;
  world_.circle(collider.shape_index).radius = parameters_[kRadius].ToScalar();

  readouts_[0].value = ToDecimal(predicted_inertia(), readouts_[0].scale);
  readouts_[1].value = ToDecimal(measured_inertia(), readouts_[1].scale);
  readouts_[2].value = ToDecimal(predicted_spin(), readouts_[2].scale);
  readouts_[4].value = ToDecimal(box_spin(), readouts_[4].scale);
  readouts_[5].value = ToDecimal(matching_radius(), readouts_[5].scale);
  for (SampleReadout& readout : readouts_) {
    readout.is_valid = true;
  }
}

void DiskSample::FixedUpdate() {
  if (is_pushing()) {
    // The couple of chapter 01: it turns with the disk, torque 2 r F
    const Body& disk = world_.body(disk_index_);
    const Vec2f lever =
        disk.rotation.x_axis() * parameters_[kLeverArm].ToScalar();
    const Vec2f force = disk.rotation.y_axis() * parameters_[kForce].ToScalar();
    world_.ApplyForceAtPoint(disk_index_, force, disk.position + lever);
    world_.ApplyForceAtPoint(disk_index_, -force, disk.position - lever);
  }
  world_.Step();
  ++step_;
  readouts_[3].value =
      ToDecimal(world_.body(disk_index_).angular_velocity, readouts_[3].scale);
}

void DiskSample::Draw(DrawList& draw_list) const {
  const Body& disk = world_.body(disk_index_);
  const Scalar radius = parameters_[kRadius].ToScalar();
  draw_list.circles.push_back({disk.position, radius, sample_color::kOrange});
  DrawOrientation(disk, radius, sample_color::kWhite, draw_list);
  if (is_pushing()) {
    const Vec2f lever =
        disk.rotation.x_axis() * parameters_[kLeverArm].ToScalar();
    const Vec2f arrow = disk.rotation.y_axis() *
                        (parameters_[kForce].ToScalar() * kForceDrawScale);
    for (const Vec2f& side : {lever, -lever}) {
      const Vec2f point = disk.position + side;
      draw_list.lines.push_back({disk.position, point, sample_color::kGrey});
      const Vec2f push = side == lever ? arrow : -arrow;
      draw_list.lines.push_back({point, point + push, sample_color::kRed});
    }
  }
}

void DiskSample::End() {
  world_.Clear();
  step_ = 0;
}

Scalar DiskSample::predicted_inertia() const {
  const Scalar radius = parameters_[kRadius].ToScalar();
  return parameters_[kMass].ToScalar() * radius * radius / Scalar{2};
}

Scalar DiskSample::measured_inertia() const {
  return Scalar{1} / world_.ComputeInverseInertia(disk_index_);
}

Scalar DiskSample::push_time() const {
  return static_cast<Scalar>(push_steps_) * world_.dt();
}

Scalar DiskSample::torque() const {
  return Scalar{2} * parameters_[kLeverArm].ToScalar() *
         parameters_[kForce].ToScalar();
}

Scalar DiskSample::predicted_spin() const {
  return torque() * push_time() / predicted_inertia();
}

Scalar DiskSample::box_spin() const {
  const Scalar box_inertia = parameters_[kMass].ToScalar() *
                             (kBoxWidth * kBoxWidth + kBoxHeight * kBoxHeight) /
                             Scalar{12};
  return torque() * push_time() / box_inertia;
}

Scalar DiskSample::matching_radius() {
  return Sqrt((kBoxWidth * kBoxWidth + kBoxHeight * kBoxHeight) / Scalar{6});
}

}  // namespace neko::tutorial
