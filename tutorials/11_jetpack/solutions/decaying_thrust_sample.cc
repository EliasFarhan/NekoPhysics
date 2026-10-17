#include "tutorials/11_jetpack/solutions/decaying_thrust_sample.h"

#include "neko/math/func.h"

namespace neko::tutorial {
namespace {
constexpr Scalar kGravity = Ratio(981, 100);
constexpr Scalar kGroundTop = Ratio(1, 2);
constexpr Scalar kPlayerRadius = Ratio(3, 10);
constexpr Vec2f kStart{Scalar{2}, Ratio(3, 2)};
// The sustained thrust of the case study, in newtons for full input
constexpr Scalar kSustained{22};
// c while rising
constexpr Scalar kRisingShare = Ratio(3, 4);
constexpr Scalar kFlamePerNewton = Ratio(1, 40);
constexpr Scalar kMillimeters{1000};
}  // namespace

void DecayingThrustSample::Begin(const SampleContext& context) {
  const Vec2f bounds_center = context.world_bounds.GetCenter();
  const Scalar half_width = context.world_bounds.GetHalfSize().x();
  ground_ = Aabbf::FromCenter({bounds_center.x(), kGroundTop / Scalar{2}},
                              {half_width, kGroundTop / Scalar{2}});
  {
    const BodyIndex ground = world_.AddBody();
    world_.body(ground).type = BodyType::kStatic;
    world_.body(ground).inverse_mass = Scalar{0};
    world_.body(ground).position = ground_.GetCenter();
    const ColliderIndex collider_index = world_.AddAabbCollider(ground);
    Collider& collider = world_.collider(collider_index);
    collider.is_trigger = false;
    collider.restitution = Scalar{0};
    world_.aabb(collider.shape_index).half_size = ground_.GetHalfSize();
  }
  player_ = world_.AddBody();
  Body& player = world_.body(player_);
  player.inverse_mass = Scalar{1};
  player.inertia_mode = InertiaMode::kFixedRotation;
  player.position = kStart;
  const ColliderIndex collider_index = world_.AddCircleCollider(player_);
  Collider& collider = world_.collider(collider_index);
  collider.is_trigger = false;
  collider.restitution = Scalar{0};
  world_.circle(collider.shape_index).radius = kPlayerRadius;
  start_y_ = kStart.y();
  decay_ = Scalar{1};
  step_ = 0;
  trail_.fill(kStart);
  readouts_[0].value = ToDecimal(burst_speed(), readouts_[0].scale);
  readouts_[0].is_valid = true;
}

Scalar DecayingThrustSample::burst_speed() const {
  // The case study's target speed B h / m, m = 1 kg
  return parameters_[kBurst].ToScalar() * world_.dt();
}

Scalar DecayingThrustSample::decay_per_step() const {
  // exp(-x) = 1 - x + x^2/2 - x^3/6 + x^4/24 - ..., x = k h = 1/12 for the
  // defaults: the next term is under 1/65536
  const Scalar x = parameters_[kDecay].ToScalar() * world_.dt();
  const Scalar x2 = x * x;
  return Scalar{1} - x + x2 / Scalar{2} - x2 * x / Scalar{6} +
         x2 * x2 / Scalar{24};
}

void DecayingThrustSample::FixedUpdate() {
  const Body& player = world_.body(player_);
  if (step_ == 0) {
    // The burst: v0 = B h / m at once, before this step's gravity
    world_.ApplyImpulse(player_, {Scalar{0}, burst_speed()});
  }
  thrust_ = Scalar{0};
  if (step_ < kInputSteps) {
    const Scalar share =
        player.velocity.y() > Scalar{0} ? kRisingShare : Scalar{1};
    const Scalar decay = step_ < kDecaySteps ? decay_ : Scalar{1};
    thrust_ = kSustained * parameters_[kInput].ToScalar() * share * decay;
    world_.ApplyForce(player_, {Scalar{0}, thrust_});
  }
  const Scalar velocity_before = player.velocity.y();
  world_.Step();
  ++step_;
  decay_ = decay_ * decay_per_step();
  if (step_ <= kTrailLength) {
    trail_[static_cast<std::size_t>(step_ - 1)] = player.position;
  }

  if (!apex_step_.has_value() && velocity_before > Scalar{0} &&
      player.velocity.y() <= Scalar{0}) {
    apex_step_ = step_;
    apex_height_ = player.position.y() - start_y_;
    // The slide's y(t) - y0 at t = n h, exp(-k t) = q^n
    const Scalar t = static_cast<Scalar>(step_) * world_.dt();
    const Scalar k = parameters_[kDecay].ToScalar();
    const Scalar a = kSustained * parameters_[kInput].ToScalar() * kRisingShare;
    slide_height_ = burst_speed() * t + a / k * (t - (Scalar{1} - decay_) / k) -
                    kGravity * t * t / Scalar{2};
    readouts_[1].value = ToDecimal(t, readouts_[1].scale);
    readouts_[2].value = ToDecimal(apex_height_, readouts_[2].scale);
    readouts_[3].value = ToDecimal(slide_height_, readouts_[3].scale);
    readouts_[4].value = ToDecimal(
        (apex_height_ - slide_height_) * kMillimeters, readouts_[4].scale);
    for (std::size_t i = 1; i < 5; ++i) {
      readouts_[i].is_valid = true;
    }
  }
}

void DecayingThrustSample::Draw(DrawList& draw_list) const {
  draw_list.boxes.push_back(
      {ground_.GetCenter(), ground_.GetHalfSize(), sample_color::kGrey});
  const int shown = step_ < kTrailLength ? step_ : kTrailLength;
  Vec2f previous = kStart;
  for (int i = 0; i < shown; ++i) {
    const Vec2f next = trail_[static_cast<std::size_t>(i)];
    // Spread the path along x so the rise and the fall don't overlap
    const Scalar x_of_i = kStart.x() + static_cast<Scalar>(i) / Scalar{20};
    const Scalar x_of_previous =
        kStart.x() + static_cast<Scalar>(i > 0 ? i - 1 : 0) / Scalar{20};
    draw_list.lines.push_back({Vec2f{x_of_previous, previous.y()},
                               Vec2f{x_of_i, next.y()}, sample_color::kWhite});
    previous = next;
  }
  const Body& player = world_.body(player_);
  if (thrust_ > Scalar{0}) {
    const Vec2f nozzle = player.position - Vec2f{Scalar{0}, kPlayerRadius};
    draw_list.lines.push_back(
        {nozzle, nozzle - Vec2f{Scalar{0}, thrust_ * kFlamePerNewton},
         sample_color::kRed});
  }
  draw_list.circles.push_back(
      {player.position, kPlayerRadius, sample_color::kOrange});
}

void DecayingThrustSample::End() {
  world_.Clear();
  step_ = 0;
  decay_ = Scalar{1};
  thrust_ = Scalar{0};
  apex_step_.reset();
  apex_height_ = Scalar{0};
  slide_height_ = Scalar{0};
  for (auto& readout : readouts_) {
    readout.is_valid = false;
  }
}

}  // namespace neko::tutorial
