#include "tutorials/10_explosion/solutions/laser_bounce_sample.h"

#include "neko/math/angle.h"
#include "neko/math/func.h"

#include "main/samples/body_draw.h"

namespace neko::tutorial {
namespace {
constexpr Vec2f kStart{Scalar{1}, Scalar{4}};
// Length of each segment: longer than the room's diagonal
constexpr Scalar kSegmentLength{30};
constexpr Scalar kNormalLength = Ratio(1, 2);
constexpr Vec2f kPostCenter{Scalar{8}, Scalar{3}};
constexpr Scalar kPostRadius = Ratio(1, 2);

struct BoxSpec {
  Vec2f center;
  Vec2f half_size;
  Scalar degrees;
};
// Floor, ceiling, left and right walls, and the tilted mirror
constexpr std::array<BoxSpec, 5> kBoxes{{
    {{Ratio(32, 5), Ratio(1, 4)}, {Ratio(32, 5), Ratio(1, 4)}, Scalar{0}},
    {{Ratio(32, 5), Ratio(139, 20)}, {Ratio(32, 5), Ratio(1, 4)}, Scalar{0}},
    {{Ratio(3, 20), Ratio(18, 5)}, {Ratio(3, 20), Ratio(18, 5)}, Scalar{0}},
    {{Ratio(253, 20), Ratio(18, 5)}, {Ratio(3, 20), Ratio(18, 5)}, Scalar{0}},
    {{Scalar{3}, Ratio(21, 5)}, {Ratio(9, 10), Ratio(1, 10)}, Scalar{60}},
}};
}  // namespace

void LaserBounceSample::AddBox(Vec2f center, Vec2f half_size, Scalar degrees) {
  const BodyIndex body = world_.AddBody();
  world_.body(body).type = BodyType::kStatic;
  world_.body(body).inverse_mass = Scalar{0};
  world_.body(body).position = center;
  ColliderIndex collider_index{};
  if (degrees == Scalar{0}) {
    collider_index = world_.AddAabbCollider(body);
    world_.aabb(world_.collider(collider_index).shape_index).half_size =
        half_size;
  } else {
    world_.body(body).rotation.set_angle(Radian{Degree{degrees}});
    collider_index = world_.AddOrientedBoxCollider(body);
    world_.oriented_box(world_.collider(collider_index).shape_index).half_size =
        half_size;
  }
  world_.collider(collider_index).is_trigger = false;
}

void LaserBounceSample::Begin([[maybe_unused]] const SampleContext& context) {
  for (const BoxSpec& box : kBoxes) {
    AddBox(box.center, box.half_size, box.degrees);
  }
  const BodyIndex post = world_.AddBody();
  world_.body(post).type = BodyType::kStatic;
  world_.body(post).inverse_mass = Scalar{0};
  world_.body(post).position = kPostCenter;
  const ColliderIndex collider_index = world_.AddCircleCollider(post);
  world_.collider(collider_index).is_trigger = false;
  world_.circle(world_.collider(collider_index).shape_index).radius =
      kPostRadius;
  Trace();
}

Vec2f LaserBounceSample::start() const { return kStart; }

void LaserBounceSample::Trace() {
  const Radian angle{Degree{static_cast<Scalar>(parameters_[kAngle].value)}};
  Vec2f direction{Cos(angle), Sin(angle)};
  // The push off the surface, in meters
  const Scalar push_off = static_cast<Scalar>(parameters_[kPushOff].value) /
                          static_cast<Scalar>(1000);
  const std::size_t wanted =
      static_cast<std::size_t>(parameters_[kBounces].value);
  Vec2f from = kStart;
  bounce_count_ = 0;
  path_length_ = Scalar{0};
  while (bounce_count_ < wanted) {
    const Vec2f to = from + direction * kSegmentLength;
    const auto hit = world_.RaycastClosest(from, to);
    if (!hit.has_value()) {
      last_end_ = to;
      break;
    }
    bounces_[bounce_count_++] = {hit->point, hit->normal, hit->fraction};
    path_length_ += hit->fraction * kSegmentLength;
    last_end_ = hit->point;
    // Reflect about the normal; a zero normal (a start inside the shape)
    // leaves the direction unchanged
    direction -= hit->normal * (Scalar{2} * Vec2f::Dot(direction, hit->normal));
    from = hit->point + direction * push_off;
  }
  const std::size_t shown = bounce_count_ < 2 ? bounce_count_ : 2;
  for (std::size_t i = 0; i < 2; ++i) {
    readouts_[2 * i].is_valid = i < shown;
    readouts_[2 * i + 1].is_valid = i < shown;
    if (i < shown) {
      readouts_[2 * i].value =
          ToDecimal(bounces_[i].point.x(), readouts_[2 * i].scale);
      readouts_[2 * i + 1].value =
          ToDecimal(bounces_[i].point.y(), readouts_[2 * i + 1].scale);
    }
  }
  readouts_[4].value = static_cast<int>(bounce_count_);
  readouts_[5].value = ToDecimal(path_length_, readouts_[5].scale);
  readouts_[4].is_valid = true;
  readouts_[5].is_valid = true;
}

void LaserBounceSample::FixedUpdate() { world_.Step(); }

void LaserBounceSample::Draw(DrawList& draw_list) const {
  for (const BoxSpec& box : kBoxes) {
    if (box.degrees == Scalar{0}) {
      draw_list.boxes.push_back(
          {box.center, box.half_size, sample_color::kGrey});
    } else {
      Rotation rotation;
      rotation.set_angle(Radian{Degree{box.degrees}});
      DrawBoxOutline(box.center, box.half_size, rotation, sample_color::kCyan,
                     draw_list);
    }
  }
  draw_list.circles.push_back({kPostCenter, kPostRadius, sample_color::kGrey});
  Vec2f from = kStart;
  for (const Bounce& bounce : bounces()) {
    draw_list.lines.push_back({from, bounce.point, sample_color::kWhite});
    draw_list.lines.push_back({bounce.point,
                               bounce.point + bounce.normal * kNormalLength,
                               sample_color::kYellow});
    from = bounce.point;
  }
  if (bounce_count_ < static_cast<std::size_t>(parameters_[kBounces].value)) {
    draw_list.lines.push_back({from, last_end_, sample_color::kWhite});
  }
  draw_list.circles.push_back({kStart, Ratio(1, 10), sample_color::kRed});
}

void LaserBounceSample::End() {
  world_.Clear();
  bounce_count_ = 0;
  path_length_ = Scalar{0};
  for (auto& readout : readouts_) {
    readout.is_valid = false;
  }
}

}  // namespace neko::tutorial
