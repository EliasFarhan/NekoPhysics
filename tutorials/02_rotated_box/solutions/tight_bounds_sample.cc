#include "tutorials/02_rotated_box/solutions/tight_bounds_sample.h"

#include "neko/math/func.h"
#include "neko/math/rotation.h"
#include "neko/math/shape.h"

#include "main/samples/body_draw.h"

namespace neko::tutorial {
namespace {
constexpr Scalar kProbeRadius = Ratio(1, 10);
// The probe's distance beyond the box's reach
constexpr Scalar kProbeGap = Ratio(1, 5);
}  // namespace

void TightBoundsSample::Begin(const SampleContext& context) {
  const Vec2f center = context.world_bounds.GetCenter();
  box_index_ = world_.AddBody();
  {
    Body& box = world_.body(box_index_);
    box.position = center;
    box.inverse_mass = Scalar{1};
    box.angular_velocity = parameters_[kSpin].ToScalar();
    const ColliderIndex collider_index =
        world_.AddOrientedBoxCollider(box_index_);
    Collider& collider = world_.collider(collider_index);
    collider.is_trigger = false;
    world_.oriented_box(collider.shape_index).half_size = half_size();
  }

  probe_index_ = world_.AddBody();
  {
    Body& probe = world_.body(probe_index_);
    probe.type = BodyType::kStatic;
    probe.inverse_mass = Scalar{0};
    // On the diagonal (1, 1) / sqrt(2), out of the box's reach
    const Scalar distance = half_diagonal() + kProbeGap;
    const Scalar along = distance / Sqrt(Scalar{2});
    probe.position = center + Vec2f{along, along};
    const ColliderIndex collider_index = world_.AddCircleCollider(probe_index_);
    Collider& collider = world_.collider(collider_index);
    // A trigger: it reports overlaps without pushing the box
    collider.is_trigger = true;
    world_.circle(collider.shape_index).radius = kProbeRadius;
  }

  readouts_[3].value = ToDecimal(half_diagonal(), readouts_[3].scale);
  for (SampleReadout& readout : readouts_) {
    readout.is_valid = true;
  }
}

void TightBoundsSample::FixedUpdate() {
  // The broadphase of this step sees the rotation of its start
  const Vec2f extents = predicted_extents();
  world_.Step();
  ++steps_;

  const Vec2f center = world_.body(box_index_).position;
  const Vec2f probe = world_.body(probe_index_).position;
  const Aabbf bounds = Aabbf::FromCenter(center, extents);
  const Aabbf probe_bounds =
      Aabbf::FromCenter(probe, Vec2f{kProbeRadius, kProbeRadius});
  const bool predicted = Intersect(bounds, probe_bounds);
  is_paired_ = world_.BroadphasePairCount() > 0;
  if (predicted != is_paired_) {
    ++mismatches_;
  }
  if (is_paired_) {
    ++paired_steps_;
  }
  if (!world_.touching_pairs().empty()) {
    ++touching_steps_;
  }
  widest_ = Max(widest_, extents.x());

  const Vec2f now = predicted_extents();
  readouts_[0].value = ToDecimal(now.x(), readouts_[0].scale);
  readouts_[1].value = ToDecimal(now.y(), readouts_[1].scale);
  readouts_[2].value = ToDecimal(corner_extents().x(), readouts_[2].scale);
  readouts_[4].value = paired_steps_;
  readouts_[5].value = touching_steps_;
  readouts_[6].value = mismatches_;
}

void TightBoundsSample::Draw(DrawList& draw_list) const {
  const Body& box = world_.body(box_index_);
  DrawBoxOutline(box.position, half_size(), box.rotation, sample_color::kOrange,
                 draw_list);
  DrawBoxOutline(box.position, predicted_extents(), Rotation{},
                 sample_color::kGrey, draw_list);
  draw_list.circles.push_back(
      {world_.body(probe_index_).position, kProbeRadius,
       is_paired_ ? sample_color::kRed : sample_color::kGreen});
}

void TightBoundsSample::End() {
  world_.Clear();
  widest_ = Scalar{0};
  is_paired_ = false;
  steps_ = 0;
  paired_steps_ = 0;
  touching_steps_ = 0;
  mismatches_ = 0;
}

Vec2f TightBoundsSample::ExtentsFor(const Rotation& rotation) const {
  const Vec2f half = half_size();
  const Scalar cos = Abs(rotation.cos());
  const Scalar sin = Abs(rotation.sin());
  return {cos * half.x() + sin * half.y(), sin * half.x() + cos * half.y()};
}

Vec2f TightBoundsSample::predicted_extents() const {
  return ExtentsFor(world_.body(box_index_).rotation);
}

Vec2f TightBoundsSample::corner_extents() const {
  const Rotation& rotation = world_.body(box_index_).rotation;
  const Vec2f half = half_size();
  Vec2f extents{};
  for (const Vec2f corner :
       {half, Vec2f{-half.x(), half.y()}, -half, Vec2f{half.x(), -half.y()}}) {
    const Vec2f turned = rotation.Rotate(corner);
    extents = {Max(extents.x(), turned.x()), Max(extents.y(), turned.y())};
  }
  return extents;
}

Scalar TightBoundsSample::half_diagonal() const { return half_size().Length(); }

Vec2f TightBoundsSample::half_size() const {
  return {parameters_[kWidth].ToScalar() / Scalar{2},
          parameters_[kHeight].ToScalar() / Scalar{2}};
}

}  // namespace neko::tutorial
