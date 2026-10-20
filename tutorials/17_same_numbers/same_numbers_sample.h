#ifndef TUTORIALS_17_SAME_NUMBERS_SAME_NUMBERS_SAMPLE_H_
#define TUTORIALS_17_SAME_NUMBERS_SAME_NUMBERS_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

#include "neko/physics/physics.h"

#include "main/samples/sample.h"

namespace neko::tutorial {

// value moved by units steps of its last bit: the next representable
// numbers (one unit is 1/65536 for Fixed16, one ulp for float and the sixit
// float). Value must be positive, so the bits grow with the value.
[[nodiscard]] Scalar NudgeBits(Scalar value, int units);

// A pachinko board, shared by the chapter and its solutions. Balls fall one
// after the other through seven staggered rows of pegs into ten bins. Every
// bounce off a round peg turns a small difference in where the ball hits
// into a larger difference in where it goes: the board amplifies, like any
// pile of bouncing bodies, and like Splash's players colliding.
//
// Ball b is released at step kDropPeriod * b from the top middle, a few
// centimeters to the side (the same seeded offsets on every board). The
// first ball can be nudged: its start x moved by a number of units in the
// last place (NudgeBits). Two boards built with different nudges are the
// same simulation except for that one number.
class Board {
 public:
  static constexpr int kBallCount = 12;
  static constexpr int kDropPeriod = 20;
  static constexpr int kRowCount = 7;
  static constexpr int kBinCount = 10;

  // Builds the board in world_bounds (the samples' 12.8 x 7.2 m view);
  // nudge moves the first ball's start x
  void Build(const Aabbf& world_bounds, int nudge);
  // Drops the ball due at this step, then steps the world
  void Step();
  void Clear();

  [[nodiscard]] PhysicsWorld& world() { return world_; }
  [[nodiscard]] const PhysicsWorld& world() const { return world_; }
  [[nodiscard]] int step() const { return step_; }
  // Balls released so far
  [[nodiscard]] int ball_count() const { return ball_count_; }
  // Bodies in the world: the static ones (floor, dividers, walls, pegs)
  // first, then the balls. Each body has one collider, of the same index.
  [[nodiscard]] int body_count() const { return static_count_ + ball_count_; }
  [[nodiscard]] Vec2f BallPosition(int ball) const;
  [[nodiscard]] BodyIndex ball_body(int ball) const {
    return balls_[static_cast<std::size_t>(ball)];
  }
  // The bin a ball has fallen into (0 to kBinCount - 1), or nothing while
  // it is above the bins
  [[nodiscard]] std::optional<int> Bin(int ball) const;
  // Where the first ball starts (after the nudge)
  [[nodiscard]] Vec2f first_start() const { return first_start_; }
  // The collider of the top peg under the drop point, which every ball
  // meets first (an exercise nudges it)
  [[nodiscard]] ColliderIndex top_peg() const { return top_peg_; }

  // Draws the pegs and walls in grey and the balls in ball_color, of
  // radius ball_radius_scale times their own
  void DrawStatic(DrawList& draw_list) const;
  void DrawBalls(DrawList& draw_list, ColorId ball_color,
                 Scalar ball_radius_scale) const;

  static constexpr Scalar kBallRadius = Ratio(3, 25);
  static constexpr Scalar kPegRadius = Ratio(7, 100);
  static constexpr Scalar kSpacing = Ratio(3, 5);

 private:
  void DropBall();

  PhysicsWorld world_{{.gravity = Vec2f{Scalar{0}, -Ratio(981, 100)},
                       .dt = Sample::kFixedDeltaTime,
                       .max_bodies = 128,
                       .max_colliders = 128,
                       .max_touching_pairs = 256,
                       .max_broadphase_pairs = 1024,
                       .max_joints = 0}};
  std::array<BodyIndex, kBallCount> balls_{};
  // The static shapes, for drawing (built once in Build)
  std::vector<DrawCircle> pegs_;
  std::vector<DrawBox> boxes_;
  std::array<Scalar, kBallCount> offsets_{};
  Vec2f drop_point_{};
  Vec2f first_start_{};
  Scalar bins_left_{};
  Scalar bins_top_{};
  ColliderIndex top_peg_ = kInvalidColliderIndex;
  int nudge_ = 0;
  int static_count_ = 0;
  int step_ = 0;
  int ball_count_ = 0;
};

// Follows two boards that should be the same and reports when they stop
// being so: the first step their world checksums differ, the first step a
// ball is more than 1 mm from its twin (visible), the largest distance so
// far, and how many balls landed in another bin.
class TwinWatch {
 public:
  void Reset();
  // After both boards stepped
  void Update(const Board& a, const Board& b);

  [[nodiscard]] std::optional<int> first_checksum_step() const {
    return first_checksum_step_;
  }
  [[nodiscard]] std::optional<int> first_visible_step() const {
    return first_visible_step_;
  }
  [[nodiscard]] Scalar max_gap() const { return max_gap_; }
  [[nodiscard]] int bins_differ() const { return bins_differ_; }
  // Distance of each ball from its twin, last update
  [[nodiscard]] Scalar Gap(int ball) const {
    return gaps_[static_cast<std::size_t>(ball)];
  }

  static constexpr Scalar kVisible = Ratio(1, 1000);

 private:
  std::array<Scalar, Board::kBallCount> gaps_{};
  std::optional<int> first_checksum_step_;
  std::optional<int> first_visible_step_;
  Scalar max_gap_{};
  int bins_differ_ = 0;
};

// Chapter 17, same numbers everywhere: one changed bit, two different games.
//
// Two copies of the same pachinko board run side by side, drawn on top of
// each other: board A in sky blue, board B in orange (smaller, over A). On
// board B the first ball starts "Nudge" units in the last place to the
// right: 1.5e-5 m with Fixed16, 4.8e-7 m with float or the sixit float at
// x = 6.4 m. That is the size of the difference a fused multiply-add or
// another sin implementation makes in one operation.
//
// Nothing visible happens at first: the checksums differ from step 0 (the
// nudge is part of the state), but the balls stay on top of each other.
// Each bounce off a round peg multiplies the difference, so after a few
// bounces it reaches millimeters, then the balls take different paths,
// push the others differently, and land in other bins. With a nudge of 0
// the boards stay identical forever: the simulation is deterministic, and
// on any machine that computes every operation the same way, a game that
// sends only inputs stays in sync. On a machine that rounds one operation
// differently, it doesn't.
//
// Readouts: the first step the checksums differ; the first step a ball is
// more than 1 mm from its twin (magenta line); the largest gap; the balls
// in another bin; the balls dropped.
//
// The world is y-up, in meters and seconds, gravity (0, -9.81) m/s^2.
class SameNumbersSample final : public Sample {
 public:
  void Begin(const SampleContext& context) override;
  void FixedUpdate() override;
  void Draw(DrawList& draw_list) const override;
  void End() override;
  [[nodiscard]] std::uint32_t GenerateChecksum() const override;
  [[nodiscard]] std::span<SampleParameter> parameters() override {
    return parameters_;
  }
  [[nodiscard]] std::span<const SampleReadout> readouts() const override {
    return readouts_;
  }

  [[nodiscard]] const TwinWatch& watch() const { return watch_; }
  [[nodiscard]] const Board& board_a() const { return board_a_; }
  [[nodiscard]] const Board& board_b() const { return board_b_; }

  static constexpr std::size_t kNudge = 0;

 private:
  void UpdateReadouts();

  Board board_a_;
  Board board_b_;
  TwinWatch watch_;
  std::array<SampleParameter, 1> parameters_{{
      {.name = "Nudge",
       .unit = "ulp",
       .min = 0,
       .max = 64,
       .step = 1,
       .scale = 1,
       .value = 1},
  }};
  std::array<SampleReadout, 5> readouts_{{
      {.name = "Checksums differ", .unit = "step", .scale = 1},
      {.name = "Visible (1 mm)", .unit = "step", .scale = 1},
      {.name = "Max gap", .unit = "m", .scale = 1000},
      {.name = "Other bin", .unit = "balls", .scale = 1},
      {.name = "Balls", .unit = "", .scale = 1},
  }};
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_17_SAME_NUMBERS_SAME_NUMBERS_SAMPLE_H_
