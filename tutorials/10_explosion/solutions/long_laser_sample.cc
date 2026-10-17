#include "tutorials/10_explosion/solutions/long_laser_sample.h"

#include "neko/math/func.h"

namespace neko::tutorial {
namespace {
constexpr Scalar kUpperY = Ratio(32, 5);
constexpr Scalar kLowerY{3};
constexpr Vec2f kPostCenter{Ratio(19, 2), Ratio(32, 5)};
constexpr Scalar kPostRadius = Ratio(2, 5);
constexpr Vec2f kBoxHalf{Ratio(2, 5), Ratio(2, 5)};
constexpr Vec2f kBoxCenter{Ratio(19, 2), Scalar{3}};
// Millimeters per meter
constexpr Scalar kMillimeters{1000};
}  // namespace

Scalar LongLaserSample::face_x() const { return kPostCenter.x() - kPostRadius; }

void LongLaserSample::Begin([[maybe_unused]] const SampleContext& context) {
  const BodyIndex post = world_.AddBody();
  world_.body(post).type = BodyType::kStatic;
  world_.body(post).inverse_mass = Scalar{0};
  world_.body(post).position = kPostCenter;
  const ColliderIndex post_collider = world_.AddCircleCollider(post);
  world_.collider(post_collider).is_trigger = false;
  world_.circle(world_.collider(post_collider).shape_index).radius =
      kPostRadius;
  const BodyIndex box = world_.AddBody();
  world_.body(box).type = BodyType::kStatic;
  world_.body(box).inverse_mass = Scalar{0};
  world_.body(box).position = kBoxCenter;
  const ColliderIndex box_collider = world_.AddAabbCollider(box);
  world_.collider(box_collider).is_trigger = false;
  world_.aabb(world_.collider(box_collider).shape_index).half_size = kBoxHalf;
  Cast();
}

void LongLaserSample::Cast() {
  const Scalar end_x = kStartX + length();
  circle_hit_ = world_.RaycastClosest({kStartX, kUpperY}, {end_x, kUpperY});
  box_hit_ = world_.RaycastClosest({kStartX, kLowerY}, {end_x, kLowerY});
  const Scalar distance = face_x() - kStartX;
  const auto show = [this, distance](const std::optional<RaycastHit>& hit,
                                     std::size_t first) {
    readouts_[first].is_valid = hit.has_value();
    readouts_[first + 1].is_valid = hit.has_value();
    if (hit.has_value()) {
      readouts_[first].value =
          ToDecimal(hit->fraction * length(), readouts_[first].scale);
      readouts_[first + 1].value =
          ToDecimal((hit->point.x() - kStartX - distance) * kMillimeters,
                    readouts_[first + 1].scale);
    }
  };
  show(circle_hit_, 0);
  show(box_hit_, 2);
  // L / 65536 in millimeters: L * 1000 would pass Fixed16's 32767, so
  // multiply by 1000 / 65536 = 125 / 8192, exact in both scalars
  readouts_[4].value =
      ToDecimal(length() * Ratio(125, 8192), readouts_[4].scale);
  readouts_[4].is_valid = true;
}

void LongLaserSample::FixedUpdate() { world_.Step(); }

void LongLaserSample::Draw(DrawList& draw_list) const {
  draw_list.circles.push_back({kPostCenter, kPostRadius, sample_color::kGrey});
  draw_list.boxes.push_back({kBoxCenter, kBoxHalf, sample_color::kGrey});
  for (const auto& hit : {circle_hit_, box_hit_}) {
    if (hit.has_value()) {
      draw_list.lines.push_back(
          {Vec2f{kStartX, hit->point.y()}, hit->point, sample_color::kWhite});
    }
  }
}

void LongLaserSample::End() {
  world_.Clear();
  circle_hit_.reset();
  box_hit_.reset();
  for (auto& readout : readouts_) {
    readout.is_valid = false;
  }
}

}  // namespace neko::tutorial
