#include "tutorials/13_one_way/solutions/arc_sweep_sample.h"

#include <algorithm>

#include "neko/math/angle.h"

#include "main/samples/body_draw.h"
#include "tutorials/13_one_way/one_way_sample.h"

namespace neko::tutorial {
namespace {
constexpr Vec2f kPlankHalfSize{Ratio(1, 2), Ratio(2, 25)};
constexpr Scalar kBallRadius = Ratio(3, 25);
constexpr Scalar kBallStart = Ratio(4, 5);
constexpr Scalar kBallSpeed{2};
constexpr Scalar kColumnSpacing = Ratio(9, 5);
constexpr Scalar kFirstColumn{1};
constexpr std::array<Scalar, 2> kRowHeights = {Ratio(26, 5), Ratio(9, 5)};

bool IsTouching(const PhysicsWorld& world, ColliderPair pair) {
  return std::any_of(
      world.touching_pairs().begin(), world.touching_pairs().end(),
      [pair](const TouchingPair& touching) { return touching.pair == pair; });
}
}  // namespace

void ArcSweepSample::Begin(const SampleContext& context) {
  const Vec2f origin = context.world_bounds.min_bound();
  for (std::size_t row = 0; row < 2; ++row) {
    const Radian arc{Degree{parameters_[row].ToScalar()}};
    for (std::size_t i = 0; i < kTiltCount; ++i) {
      const Vec2f center{
          origin.x() + kFirstColumn +
              kColumnSpacing * static_cast<Scalar>(static_cast<int>(i)),
          origin.y() + kRowHeights[row]};
      const Rotation tilt{Radian{Degree{static_cast<Scalar>(kTilts[i])}}};
      Shot& shot = shots_[row][i];
      const BodyIndex plank = world_.AddBody();
      world_.body(plank).type = BodyType::kStatic;
      world_.body(plank).inverse_mass = Scalar{0};
      world_.body(plank).position = center;
      world_.body(plank).rotation = tilt;
      shot.plank = world_.AddOrientedBoxCollider(plank);
      Collider& collider = world_.collider(shot.plank);
      collider.is_trigger = false;
      collider.is_one_way = true;
      collider.one_way_arc = arc;
      collider.restitution = Scalar{0};
      // Straight up in the world: the local normal is turned back by the tilt
      collider.one_way_normal = tilt.InverseRotate(Vec2f::Up());
      world_.oriented_box(collider.shape_index).half_size = kPlankHalfSize;
      // Thrown at the middle of the top face, along its normal
      const Vec2f face_normal = tilt.y_axis();
      const BodyIndex ball = world_.AddBody();
      world_.body(ball).position = center + face_normal * kBallStart;
      world_.body(ball).velocity = -face_normal * kBallSpeed;
      world_.body(ball).inertia_mode = InertiaMode::kFixedRotation;
      shot.ball = world_.AddCircleCollider(ball);
      Collider& ball_collider = world_.collider(shot.ball);
      ball_collider.is_trigger = false;
      ball_collider.restitution = Scalar{0};
      world_.circle(ball_collider.shape_index).radius = kBallRadius;
    }
  }
  for (std::size_t row = 0; row < 2; ++row) {
    readouts_[2 * row + 1].value = predicted_max_tilt(row);
    readouts_[2 * row + 1].is_valid = true;
  }
}

void ArcSweepSample::FixedUpdate() {
  world_.Step();
  ++step_;
  for (std::size_t row = 0; row < 2; ++row) {
    for (std::size_t i = 0; i < kTiltCount; ++i) {
      if (outcomes_[row][i] != 0) {
        continue;
      }
      const ColliderPair pair{shots_[row][i].plank, shots_[row][i].ball};
      if (world_.IsPassing(pair)) {
        outcomes_[row][i] = -1;
      } else if (IsTouching(world_, pair)) {
        outcomes_[row][i] = 1;
      }
    }
    const int landed = max_landed_tilt(row);
    readouts_[2 * row].value = landed;
    readouts_[2 * row].is_valid = landed >= 0;
  }
}

int ArcSweepSample::max_landed_tilt(std::size_t row) const {
  int tilt = -1;
  for (std::size_t i = 0; i < kTiltCount; ++i) {
    if (outcomes_[row][i] == 1) {
      tilt = std::max(tilt, kTilts[i]);
    }
  }
  return tilt;
}

int ArcSweepSample::predicted_max_tilt(std::size_t row) const {
  int tilt = -1;
  for (const int candidate : kTilts) {
    // Strictly within half the arc
    if (2 * candidate < parameters_[row].value) {
      tilt = std::max(tilt, candidate);
    }
  }
  return tilt;
}

void ArcSweepSample::Draw(DrawList& draw_list) const {
  for (std::size_t row = 0; row < 2; ++row) {
    for (std::size_t i = 0; i < kTiltCount; ++i) {
      const Shot& shot = shots_[row][i];
      const Collider& plank = world_.collider(shot.plank);
      DrawBoxOutline(world_.ColliderCenter(shot.plank), kPlankHalfSize,
                     world_.body(plank.body_index).rotation,
                     sample_color::kTeal, draw_list);
      DrawOneWayNormal(world_, shot.plank, draw_list);
      const ColorId color = outcomes_[row][i] == 1    ? sample_color::kGreen
                            : outcomes_[row][i] == -1 ? sample_color::kRed
                                                      : sample_color::kWhite;
      draw_list.circles.push_back(
          {world_.ColliderCenter(shot.ball), kBallRadius, color});
    }
  }
  DrawPassingPairs(world_, draw_list);
}

void ArcSweepSample::End() {
  world_.Clear();
  step_ = 0;
  outcomes_ = {};
  for (auto& readout : readouts_) {
    readout.is_valid = false;
  }
}

}  // namespace neko::tutorial
