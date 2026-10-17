#include "tutorials/05_friction/solutions/push_to_start_sample.h"

#include "main/samples/body_draw.h"

namespace neko::tutorial {
namespace {
constexpr Vec2f kGroundHalfSize{Scalar{6}, Ratio(1, 2)};
constexpr Vec2f kBoxHalfSize{Ratio(1, 2), Ratio(1, 2)};
constexpr Scalar kGravity = Ratio(981, 100);
// Length of the push line per newton, in meters
constexpr Scalar kPushDrawScale = Ratio(1, 10);
}  // namespace

void PushToStartSample::Begin(const SampleContext& context) {
  const Scalar friction = parameters_[kFriction].ToScalar();
  const Vec2f center = context.world_bounds.GetCenter();
  const Scalar left = context.world_bounds.min_bound().x();

  ground_index_ = world_.AddBody();
  {
    Body& ground = world_.body(ground_index_);
    ground.type = BodyType::kStatic;
    ground.inverse_mass = Scalar{0};
    ground.position = center - Vec2f{Scalar{0}, Scalar{2}};
    const ColliderIndex collider_index = world_.AddAabbCollider(ground_index_);
    Collider& collider = world_.collider(collider_index);
    collider.is_trigger = false;
    collider.restitution = Scalar{0};
    collider.friction = friction;
    world_.aabb(collider.shape_index).half_size = kGroundHalfSize;
  }

  box_index_ = world_.AddBody();
  {
    Body& box = world_.body(box_index_);
    box.inverse_mass = Scalar{1};
    box.position = {left + Scalar{2}, world_.body(ground_index_).position.y() +
                                          kGroundHalfSize.y() +
                                          kBoxHalfSize.y()};
    const ColliderIndex collider_index =
        world_.AddOrientedBoxCollider(box_index_);
    Collider& collider = world_.collider(collider_index);
    collider.is_trigger = false;
    collider.restitution = Scalar{0};
    collider.friction = friction;
    world_.oriented_box(collider.shape_index).half_size = kBoxHalfSize;
  }

  readouts_[0].value = ToDecimal(static_limit(), readouts_[0].scale);
  readouts_[1].value = predicted_holds() ? 1 : 0;
  readouts_[2].value = ToDecimal(predicted_acceleration(), readouts_[2].scale);
  for (std::size_t i = 0; i < 3; ++i) {
    readouts_[i].is_valid = true;
  }
}

void PushToStartSample::FixedUpdate() {
  // A force lasts one step: the world clears it after integrating it
  world_.body(box_index_).force = {parameters_[kPush].ToScalar(), Scalar{0}};
  world_.Step();
  ++step_;
  if (step_ == kMeasureSteps) {
    const Scalar time = static_cast<Scalar>(kMeasureSteps) * world_.dt();
    measured_acceleration_ = box().velocity.x() / time;
    readouts_[3].value = ToDecimal(*measured_acceleration_, readouts_[3].scale);
    readouts_[4].value = ToDecimal(*measured_friction(), readouts_[4].scale);
    readouts_[3].is_valid = true;
    readouts_[4].is_valid = true;
  }
}

Scalar PushToStartSample::static_limit() const {
  return parameters_[kFriction].ToScalar() * kGravity;  // m = 1 kg
}

bool PushToStartSample::predicted_holds() const {
  return parameters_[kPush].ToScalar() <= static_limit();
}

Scalar PushToStartSample::predicted_acceleration() const {
  if (predicted_holds()) {
    return Scalar{0};
  }
  return parameters_[kPush].ToScalar() - static_limit();  // m = 1 kg
}

std::optional<Scalar> PushToStartSample::measured_friction() const {
  if (!measured_acceleration_.has_value()) {
    return std::nullopt;
  }
  return parameters_[kPush].ToScalar() - *measured_acceleration_;  // m = 1 kg
}

void PushToStartSample::Draw(DrawList& draw_list) const {
  const Body& body = box();
  draw_list.boxes.push_back({world_.body(ground_index_).position,
                             kGroundHalfSize, sample_color::kDarkGrey});
  DrawBoxOutline(body.position, kBoxHalfSize, body.rotation,
                 sample_color::kOrange, draw_list);
  // The push, ending at the box's center
  const Vec2f push{parameters_[kPush].ToScalar() * kPushDrawScale, Scalar{0}};
  draw_list.lines.push_back(
      {body.position - push, body.position, sample_color::kRed});
  DrawOrientation(body, kBoxHalfSize.x(), sample_color::kWhite, draw_list);
  DrawVelocity(body, sample_color::kSky, draw_list);
}

void PushToStartSample::End() {
  world_.Clear();
  step_ = 0;
  measured_acceleration_.reset();
  readouts_[3].is_valid = false;
  readouts_[4].is_valid = false;
}

}  // namespace neko::tutorial
