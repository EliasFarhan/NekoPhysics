#include "tutorials/16_seesaw/solutions/pin_vs_distance_sample.h"

#include "neko/math/angle.h"
#include "neko/math/const.h"
#include "neko/math/func.h"

#include "main/samples/body_draw.h"
#include "tutorials/16_seesaw/seesaw_sample.h"

namespace neko::tutorial {
namespace {
constexpr Scalar kGravity = Ratio(981, 100);
constexpr Scalar kLength{2};
constexpr Scalar kPivotRadius = Ratio(1, 20);
constexpr Scalar kKickedRadius = Ratio(1, 10);
constexpr Scalar kKickSpeed{2};

Radian Amplitude() { return Radian{Degree{Scalar{30}}}; }

int Millimeters(Scalar meters, int scale) {
  return ToDecimal(meters * Scalar{1000}, scale);
}
}  // namespace

void PinVsDistanceSample::Begin(const SampleContext& context) {
  const Scalar center_x = context.world_bounds.GetCenter().x();
  const Scalar top = context.world_bounds.max_bound().y() - Ratio(1, 2);
  const Radian angle = Amplitude();
  for (std::size_t i = 0; i < 2; ++i) {
    const Scalar side = i == 0 ? Scalar{-1} : Scalar{1};
    const Vec2f pivot{center_x + side * Ratio(5, 2), top};
    pivots_[i] = AddAxle(world_, pivot);
    bobs_[i] = world_.AddBody();
    world_.body(bobs_[i]).position =
        pivot + Vec2f{Sin(angle), -Cos(angle)} * kLength;
    world_.body(bobs_[i]).inverse_mass = Scalar{1};
    const ColliderIndex index = world_.AddCircleCollider(bobs_[i]);
    // Solid, for its inertia m r^2 / 2
    world_.collider(index).is_trigger = false;
    world_.circle(world_.collider(index).shape_index).radius = radius();
    previous_x_[i] = world_.body(bobs_[i]).position.x() - pivot.x();

    const Vec2f kick_pivot{pivot.x(),
                           context.world_bounds.min_bound().y() + Ratio(3, 2)};
    kick_pivots_[i] = AddAxle(world_, kick_pivot);
    kicked_[i] = world_.AddBody();
    world_.body(kicked_[i]).position = kick_pivot;
    world_.body(kicked_[i]).inverse_mass = Scalar{1};
  }
  world_.AddRevoluteJoint(pivots_[0], bobs_[0],
                          world_.body(pivots_[0]).position);
  world_.AddDistanceJoint(pivots_[1], bobs_[1]);
  world_.AddRevoluteJoint(kick_pivots_[0], kicked_[0],
                          world_.body(kick_pivots_[0]).position);
  world_.AddDistanceJoint(kick_pivots_[1], kicked_[1]);

  readouts_[1].value = ToDecimal(predicted_period(0), readouts_[1].scale);
  readouts_[3].value = ToDecimal(predicted_period(1), readouts_[3].scale);
  readouts_[1].is_valid = true;
  readouts_[3].is_valid = true;
}

void PinVsDistanceSample::FixedUpdate() {
  if (step_ % kKickPeriod == 0) {
    for (const BodyIndex kicked : kicked_) {
      world_.body(kicked).velocity = {kKickSpeed, Scalar{0}};
    }
  }
  world_.Step();
  ++step_;
  const Scalar dt = world_.dt();
  for (std::size_t i = 0; i < 2; ++i) {
    const Scalar x = world_.body(bobs_[i]).position.x() -
                     world_.body(pivots_[i]).position.x();
    if ((previous_x_[i] < Scalar{0}) != (x < Scalar{0})) {
      const Scalar fraction = previous_x_[i] / (previous_x_[i] - x);
      const Scalar time = (static_cast<Scalar>(step_ - 1) + fraction) * dt;
      if (crossings_[i] == 0) {
        first_crossing_[i] = time;
      }
      last_crossing_[i] = time;
      ++crossings_[i];
    }
    previous_x_[i] = x;
    max_kick_gap_[i] =
        Max(max_kick_gap_[i], (world_.body(kicked_[i]).position -
                               world_.body(kick_pivots_[i]).position)
                                  .Length());
  }
  for (std::size_t i = 0; i < 2; ++i) {
    if (const auto period = measured_period(i)) {
      SampleReadout& readout = readouts_[2 * i];
      readout.value = ToDecimal(*period, readout.scale);
      readout.is_valid = true;
    }
    SampleReadout& gap = readouts_[4 + i];
    gap.value = Millimeters(max_kick_gap_[i], gap.scale);
    gap.is_valid = true;
  }
}

void PinVsDistanceSample::Draw(DrawList& draw_list) const {
  const std::array<ColorId, 2> colors{sample_color::kSky,
                                      sample_color::kOrange};
  for (std::size_t i = 0; i < 2; ++i) {
    const Body& pivot = world_.body(pivots_[i]);
    const Body& bob = world_.body(bobs_[i]);
    draw_list.lines.push_back({pivot.position, bob.position, colors[i]});
    draw_list.circles.push_back({bob.position, radius(), colors[i]});
    // A spoke: the pinned disk turns, the other one doesn't
    draw_list.lines.push_back({bob.position,
                               bob.position + bob.rotation.x_axis() * radius(),
                               sample_color::kWhite});
    draw_list.circles.push_back(
        {pivot.position, kPivotRadius, sample_color::kGrey});
    const Body& kick_pivot = world_.body(kick_pivots_[i]);
    draw_list.circles.push_back(
        {world_.body(kicked_[i]).position, kKickedRadius, colors[i]});
    draw_list.circles.push_back(
        {kick_pivot.position, kPivotRadius, sample_color::kGrey});
  }
}

void PinVsDistanceSample::End() {
  world_.Clear();
  step_ = 0;
  previous_x_ = {};
  first_crossing_ = {};
  last_crossing_ = {};
  crossings_ = {};
  max_kick_gap_ = {};
  for (auto& readout : readouts_) {
    readout.is_valid = false;
  }
}

std::optional<Scalar> PinVsDistanceSample::measured_period(
    std::size_t index) const {
  if (crossings_[index] < 3) {
    return std::nullopt;
  }
  // Two crossings per period
  return Scalar{2} * (last_crossing_[index] - first_crossing_[index]) /
         static_cast<Scalar>(crossings_[index] - 1);
}

Scalar PinVsDistanceSample::predicted_period(std::size_t index) const {
  const Scalar theta = Amplitude().value();
  const Scalar point = Scalar{2} * Pi<Scalar>() * Sqrt(kLength / kGravity) *
                       (Scalar{1} + theta * theta / Scalar{16});
  if (index == 1) {
    return point;
  }
  const Scalar r = radius();
  return point * Sqrt(Scalar{1} + r * r / (Scalar{2} * kLength * kLength));
}

}  // namespace neko::tutorial
