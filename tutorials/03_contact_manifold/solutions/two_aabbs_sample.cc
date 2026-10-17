#include "tutorials/03_contact_manifold/solutions/two_aabbs_sample.h"

#include "neko/math/func.h"

namespace neko::tutorial {
namespace {
constexpr Vec2f kLedgeHalfSize{Scalar{2}, Ratio(1, 2)};
constexpr Vec2f kBlockHalfSize{Ratio(1, 2), Ratio(1, 2)};
constexpr Scalar kPointRadius = Ratio(1, 16);
}  // namespace

void TwoAabbsSample::Begin(const SampleContext& context) {
  const Vec2f center = context.world_bounds.GetCenter();

  ledge_index_ = world_.AddBody();
  {
    Body& ledge = world_.body(ledge_index_);
    ledge.type = BodyType::kStatic;
    ledge.inverse_mass = Scalar{0};
    ledge.position = center - Vec2f{Scalar{1}, Ratio(3, 2)};
    ledge_collider_ = world_.AddAabbCollider(ledge_index_);
    world_.collider(ledge_collider_).is_trigger = false;
    world_.aabb(world_.collider(ledge_collider_).shape_index).half_size =
        kLedgeHalfSize;
  }

  block_index_ = world_.AddBody();
  {
    Body& block = world_.body(block_index_);
    block.type = BodyType::kKinematic;
    block.position =
        ledge_corner() +
        Vec2f{kBlockHalfSize.x() - parameters_[kOnLedge].ToScalar(),
              kBlockHalfSize.y() - parameters_[kSinkDepth].ToScalar()};
    block_collider_ = world_.AddAabbCollider(block_index_);
    world_.collider(block_collider_).is_trigger = false;
    world_.aabb(world_.collider(block_collider_).shape_index).half_size =
        kBlockHalfSize;
  }
  UpdateContact();
}

void TwoAabbsSample::FixedUpdate() {
  world_.Step();
  UpdateContact();
}

void TwoAabbsSample::UpdateContact() {
  contact_ = Contact{};
  if (!world_.DetectContact(world_.body(ledge_index_),
                            world_.collider(ledge_collider_),
                            world_.body(block_index_),
                            world_.collider(block_collider_), &contact_)) {
    contact_.point_count = 0;
  }

  const Vec2f normal = predicted_normal();
  const std::array<Vec2f, 2> points = predicted_points();
  const Vec2f corner = ledge_corner();
  readouts_[0].value = ToDecimal(normal.x(), readouts_[0].scale);
  readouts_[2].value = ToDecimal(normal.y(), readouts_[2].scale);
  readouts_[4].value = ToDecimal(predicted_penetration(), readouts_[4].scale);
  for (std::size_t i = 0; i < 2; ++i) {
    readouts_[6 + 4 * i].value =
        ToDecimal(points[i].x() - corner.x(), readouts_[6 + 4 * i].scale);
    readouts_[7 + 4 * i].value =
        ToDecimal(points[i].y() - corner.y(), readouts_[7 + 4 * i].scale);
    readouts_[6 + 4 * i].is_valid = true;
    readouts_[7 + 4 * i].is_valid = true;
  }
  readouts_[0].is_valid = true;
  readouts_[2].is_valid = true;
  readouts_[4].is_valid = true;

  const bool touching = contact_.point_count > 0;
  readouts_[1].value =
      ToDecimal(contact_.contact_normal.x(), readouts_[1].scale);
  readouts_[3].value =
      ToDecimal(contact_.contact_normal.y(), readouts_[3].scale);
  readouts_[5].value = ToDecimal(contact_.penetration, readouts_[5].scale);
  readouts_[1].is_valid = touching;
  readouts_[3].is_valid = touching;
  readouts_[5].is_valid = touching;
  for (std::size_t i = 0; i < 2; ++i) {
    const bool has_point = i < contact_.point_count;
    const Vec2f point = contact_.points[i].position;
    readouts_[8 + 4 * i].value =
        ToDecimal(point.x() - corner.x(), readouts_[8 + 4 * i].scale);
    readouts_[9 + 4 * i].value =
        ToDecimal(point.y() - corner.y(), readouts_[9 + 4 * i].scale);
    readouts_[8 + 4 * i].is_valid = has_point;
    readouts_[9 + 4 * i].is_valid = has_point;
  }
}

Vec2f TwoAabbsSample::predicted_normal() const {
  // The smaller overlap wins, y on a tie; the block is right of and above
  // the ledge's center, so the normal toward the ledge is left or down
  return parameters_[kOnLedge].value < parameters_[kSinkDepth].value
             ? Vec2f::Left()
             : Vec2f::Down();
}

Scalar TwoAabbsSample::predicted_penetration() const {
  return Min(parameters_[kOnLedge].ToScalar(),
             parameters_[kSinkDepth].ToScalar());
}

std::array<Vec2f, 2> TwoAabbsSample::predicted_points() const {
  const Vec2f ledge = world_.body(ledge_index_).position;
  const Vec2f block = world_.body(block_index_).position;
  if (predicted_normal() == Vec2f::Down()) {
    // On the block's bottom face, from the block's left side to the ledge's
    // right side
    const Scalar y = block.y() - kBlockHalfSize.y();
    return {Vec2f{Max(ledge.x() - kLedgeHalfSize.x(),
                      block.x() - kBlockHalfSize.x()),
                  y},
            Vec2f{Min(ledge.x() + kLedgeHalfSize.x(),
                      block.x() + kBlockHalfSize.x()),
                  y}};
  }
  // On the block's left face, from the block's bottom to the ledge's top
  const Scalar x = block.x() - kBlockHalfSize.x();
  return {Vec2f{x, Max(ledge.y() - kLedgeHalfSize.y(),
                       block.y() - kBlockHalfSize.y())},
          Vec2f{x, Min(ledge.y() + kLedgeHalfSize.y(),
                       block.y() + kBlockHalfSize.y())}};
}

Vec2f TwoAabbsSample::ledge_corner() const {
  return world_.body(ledge_index_).position + kLedgeHalfSize;
}

void TwoAabbsSample::Draw(DrawList& draw_list) const {
  draw_list.boxes.push_back({world_.body(ledge_index_).position, kLedgeHalfSize,
                             sample_color::kDarkGrey});
  draw_list.boxes.push_back({world_.body(block_index_).position, kBlockHalfSize,
                             sample_color::kOrange});
  for (const Vec2f point : predicted_points()) {
    draw_list.circles.push_back(
        {point, kPointRadius * Scalar{2}, sample_color::kGreen});
  }
  for (std::uint8_t i = 0; i < contact_.point_count; ++i) {
    draw_list.circles.push_back(
        {contact_.points[i].position, kPointRadius, sample_color::kYellow});
  }
  if (contact_.point_count > 0) {
    const Vec2f from = contact_.points[0].position;
    draw_list.lines.push_back(
        {from, from + contact_.contact_normal, sample_color::kWhite});
  }
}

void TwoAabbsSample::End() {
  world_.Clear();
  contact_ = Contact{};
}

}  // namespace neko::tutorial
