#include "tutorials/06_stacking/solutions/heavy_on_light_sample.h"

#include "main/samples/body_draw.h"

namespace neko::tutorial {
namespace {
constexpr Vec2f kGroundHalfSize{Scalar{6}, Ratio(1, 2)};
constexpr Vec2f kBoxHalfSize{Ratio(1, 2), Ratio(1, 2)};
// The two stacks stand this far left and right of the center, in meters
constexpr Scalar kStackOffset{2};
constexpr BodyIndex kTopBox{HeavyOnLightSample::kBoxes};

PhysicsWorld MakeWorld(int passes, bool warm_start) {
  const auto bodies = static_cast<std::size_t>(HeavyOnLightSample::kBoxes);
  return PhysicsWorld{{.gravity = Vec2f{Scalar{0}, -Ratio(981, 100)},
                       .dt = Sample::kFixedDeltaTime,
                       .max_bodies = bodies + 1,
                       .max_colliders = bodies + 1,
                       .max_touching_pairs = bodies,
                       .max_broadphase_pairs = 2 * bodies,
                       .velocity_iterations = passes,
                       .warm_starting = warm_start}};
}

// A ground and a stack of AABB boxes at x, the top one of top_mass kg
void BuildStack(PhysicsWorld& world, Scalar x, Scalar bottom, Scalar top_mass) {
  const BodyIndex ground = world.AddBody();
  world.body(ground).type = BodyType::kStatic;
  world.body(ground).inverse_mass = Scalar{0};
  world.body(ground).position = {x, bottom + kGroundHalfSize.y()};
  const ColliderIndex ground_collider = world.AddAabbCollider(ground);
  world.collider(ground_collider).is_trigger = false;
  world.collider(ground_collider).restitution = Scalar{0};
  world.aabb(world.collider(ground_collider).shape_index).half_size = {
      Ratio(3, 2), kGroundHalfSize.y()};
  const Scalar ground_top = bottom + kGroundHalfSize.y() * Scalar{2};
  for (int i = 0; i < HeavyOnLightSample::kBoxes; ++i) {
    const BodyIndex box = world.AddBody();
    world.body(box).position = {
        x, ground_top + kBoxHalfSize.y() +
               static_cast<Scalar>(i) * kBoxHalfSize.y() * Scalar{2}};
    world.body(box).inverse_mass =
        i == HeavyOnLightSample::kBoxes - 1 ? Scalar{1} / top_mass : Scalar{1};
    const ColliderIndex collider = world.AddAabbCollider(box);
    world.collider(collider).is_trigger = false;
    world.collider(collider).restitution = Scalar{0};
    world.aabb(world.collider(collider).shape_index).half_size = kBoxHalfSize;
  }
}

Scalar TopSink(const PhysicsWorld& world, Scalar ground_top) {
  const Scalar perfect = ground_top + kBoxHalfSize.y() +
                         static_cast<Scalar>(HeavyOnLightSample::kBoxes - 1) *
                             kBoxHalfSize.y() * Scalar{2};
  return perfect - world.body(kTopBox).position.y();
}

void DrawStack(const PhysicsWorld& world, ColorId color, ColorId top_color,
               DrawList& draw_list) {
  const Body& ground = world.body(BodyIndex{0});
  draw_list.boxes.push_back({ground.position,
                             {Ratio(3, 2), kGroundHalfSize.y()},
                             sample_color::kDarkGrey});
  for (int i = 0; i < HeavyOnLightSample::kBoxes; ++i) {
    const Body& box = world.body(BodyIndex{static_cast<Index>(i + 1)});
    DrawBoxOutline(box.position, kBoxHalfSize, box.rotation,
                   i == HeavyOnLightSample::kBoxes - 1 ? top_color : color,
                   draw_list);
  }
}
}  // namespace

void HeavyOnLightSample::Begin(const SampleContext& context) {
  const int passes = parameters_[kPasses].value;
  const bool warm_start = parameters_[kWarmStart].value != 0;
  world_ = MakeWorld(passes, warm_start);
  reference_ = MakeWorld(passes, warm_start);
  const Vec2f center = context.world_bounds.GetCenter();
  const Scalar bottom = context.world_bounds.min_bound().y();
  ground_top_ = bottom + kGroundHalfSize.y() * Scalar{2};
  BuildStack(world_, center.x() - kStackOffset, bottom,
             static_cast<Scalar>(parameters_[kTopMass].value));
  BuildStack(reference_, center.x() + kStackOffset, bottom, Scalar{1});
  UpdateReadouts();
}

void HeavyOnLightSample::FixedUpdate() {
  world_.Step();
  reference_.Step();
  UpdateReadouts();
}

void HeavyOnLightSample::UpdateReadouts() {
  readouts_[0].value = ToDecimal(top_sink(), readouts_[0].scale);
  readouts_[1].value = ToDecimal(reference_top_sink(), readouts_[1].scale);
  readouts_[0].is_valid = true;
  readouts_[1].is_valid = true;
}

Scalar HeavyOnLightSample::top_sink() const {
  return TopSink(world_, ground_top_);
}

Scalar HeavyOnLightSample::reference_top_sink() const {
  return TopSink(reference_, ground_top_);
}

void HeavyOnLightSample::Draw(DrawList& draw_list) const {
  DrawStack(world_, sample_color::kOrange, sample_color::kRed, draw_list);
  DrawStack(reference_, sample_color::kGrey, sample_color::kGrey, draw_list);
}

void HeavyOnLightSample::End() {
  world_.Clear();
  reference_.Clear();
}

}  // namespace neko::tutorial
