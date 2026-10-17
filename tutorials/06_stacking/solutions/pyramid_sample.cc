#include "tutorials/06_stacking/solutions/pyramid_sample.h"

#include "neko/math/func.h"

#include "main/samples/body_draw.h"

namespace neko::tutorial {
namespace {
constexpr Vec2f kGroundHalfSize{Scalar{6}, Ratio(1, 2)};
constexpr Vec2f kBoxHalfSize{Ratio(1, 2), Ratio(1, 2)};
// Distance between the centers of two neighbors in a row: 1 m box, 0.2 m gap
constexpr Scalar kSpacing = Ratio(6, 5);

BodyIndex BoxIndex(int box) { return BodyIndex{static_cast<Index>(box + 1)}; }
}  // namespace

int PyramidSample::box_count() const {
  const int rows = parameters_[kRows].value;
  return rows * (rows + 1) / 2;
}

void PyramidSample::Begin(const SampleContext& context) {
  world_ = PhysicsWorld{
      {.gravity = Vec2f{Scalar{0}, -Ratio(981, 100)},
       .dt = kFixedDeltaTime,
       .max_bodies = static_cast<std::size_t>(kMaxBoxes + 1),
       .max_colliders = static_cast<std::size_t>(kMaxBoxes + 1),
       .max_touching_pairs = static_cast<std::size_t>(3 * kMaxBoxes),
       .max_broadphase_pairs = static_cast<std::size_t>(6 * kMaxBoxes),
       .velocity_iterations = parameters_[kPasses].value,
       .warm_starting = parameters_[kWarmStart].value != 0}};

  const Vec2f center = context.world_bounds.GetCenter();
  const Scalar bottom = context.world_bounds.min_bound().y();
  const BodyIndex ground = world_.AddBody();
  world_.body(ground).type = BodyType::kStatic;
  world_.body(ground).inverse_mass = Scalar{0};
  world_.body(ground).position = {center.x(), bottom + kGroundHalfSize.y()};
  const ColliderIndex ground_collider = world_.AddAabbCollider(ground);
  world_.collider(ground_collider).is_trigger = false;
  world_.collider(ground_collider).restitution = Scalar{0};
  world_.aabb(world_.collider(ground_collider).shape_index).half_size =
      kGroundHalfSize;
  const Scalar ground_top = bottom + kGroundHalfSize.y() * Scalar{2};

  const int rows = parameters_[kRows].value;
  int box = 0;
  for (int row = 0; row < rows; ++row) {
    const int count = rows - row;
    for (int i = 0; i < count; ++i) {
      // Centered on the scene: offsets -(count - 1)/2 ... (count - 1)/2
      const Scalar offset =
          (static_cast<Scalar>(2 * i - (count - 1)) * kSpacing) / Scalar{2};
      const BodyIndex index = world_.AddBody();
      world_.body(index).position = {
          center.x() + offset,
          ground_top + kBoxHalfSize.y() +
              static_cast<Scalar>(row) * kBoxHalfSize.y() * Scalar{2}};
      starts_[static_cast<std::size_t>(box)] = world_.body(index).position;
      const ColliderIndex collider = world_.AddOrientedBoxCollider(index);
      world_.collider(collider).is_trigger = false;
      world_.collider(collider).restitution = Scalar{0};
      world_.oriented_box(world_.collider(collider).shape_index).half_size =
          kBoxHalfSize;
      ++box;
    }
  }
  UpdateReadouts();
}

void PyramidSample::FixedUpdate() {
  world_.Step();
  UpdateReadouts();
}

void PyramidSample::UpdateReadouts() {
  readouts_[0].value = ToDecimal(top_sink(), readouts_[0].scale);
  readouts_[1].value = ToDecimal(drift(), readouts_[1].scale);
  readouts_[2].value = fallen();
  for (auto& readout : readouts_) {
    readout.is_valid = true;
  }
}

Scalar PyramidSample::top_sink() const {
  const int top = box_count() - 1;
  return starts_[static_cast<std::size_t>(top)].y() -
         world_.body(BoxIndex(top)).position.y();
}

Scalar PyramidSample::drift() const {
  Scalar largest{};
  for (int i = 0; i < box_count(); ++i) {
    largest = Max(largest, Abs(world_.body(BoxIndex(i)).position.x() -
                               starts_[static_cast<std::size_t>(i)].x()));
  }
  return largest;
}

bool PyramidSample::has_fallen(int box) const {
  return starts_[static_cast<std::size_t>(box)].y() -
             world_.body(BoxIndex(box)).position.y() >
         Ratio(1, 2);
}

int PyramidSample::fallen() const {
  int count = 0;
  for (int i = 0; i < box_count(); ++i) {
    count += has_fallen(i) ? 1 : 0;
  }
  return count;
}

void PyramidSample::Draw(DrawList& draw_list) const {
  const Body& ground = world_.body(BodyIndex{0});
  draw_list.boxes.push_back(
      {ground.position, kGroundHalfSize, sample_color::kDarkGrey});
  for (int i = 0; i < box_count(); ++i) {
    const Body& box = world_.body(BoxIndex(i));
    DrawBoxOutline(box.position, kBoxHalfSize, box.rotation,
                   has_fallen(i) ? sample_color::kRed : sample_color::kOrange,
                   draw_list);
  }
}

void PyramidSample::End() { world_.Clear(); }

}  // namespace neko::tutorial
