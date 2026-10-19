#ifndef TUTORIALS_16_SEESAW_SOLUTIONS_OVER_CONSTRAINED_SAMPLE_H_
#define TUTORIALS_16_SEESAW_SOLUTIONS_OVER_CONSTRAINED_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "neko/physics/physics.h"

#include "main/samples/sample.h"

namespace neko::tutorial {

// Chapter 16, exercise "over-constrained": the LuchaSteps rig of Splash,
// whose joints ask for more than the bodies can give.
//
// An 8 m anchor beam on a static axle, two balances (4 m) hanging 2 m under
// its ends, two 4 kg platforms hanging from the outer ends of the balances:
// five revolute joints limited to plus or minus 45 degrees. Two rigid
// distance joints tie the axle to the platforms, 7.5 m away when the rig is
// built, with the length "Rod length". At 7.5 m they ask for nothing new; at
// 8 m (the level's value) the hinges and their limits can't let the
// platforms that far, so the solver splits the error: the rods stay a little
// short and the hinges come apart by a few centimeters, for as long as the
// rig exists. Nothing explodes: each joint is a soft step, an implicit
// spring that only pulls back what it can. More solver passes ("Passes")
// share the error differently, not away.
//
// Readouts over the last second: the largest gap between the two anchors of
// a hinge, the largest rod length error, the largest body speed, and the
// anchor beam's distance from its axle.
//
// Drawn: the axle and the hinges in grey, the beams in white, the platforms
// in sky blue, the rods in orange (red while more than 1 cm short).
//
// The world is y-up, in meters and seconds, gravity (0, -9.81) m/s^2.
class OverConstrainedSample final : public Sample {
 public:
  void Begin(const SampleContext& context) override;
  void FixedUpdate() override;
  void Draw(DrawList& draw_list) const override;
  void End() override;
  [[nodiscard]] std::uint32_t GenerateChecksum() const override {
    return world_->GenerateChecksum();
  }
  [[nodiscard]] std::span<SampleParameter> parameters() override {
    return parameters_;
  }
  [[nodiscard]] std::span<const SampleReadout> readouts() const override {
    return readouts_;
  }

  // Over the last kWindowSteps steps (the current window once it is full)
  [[nodiscard]] Scalar max_hinge_gap() const { return window_[0]; }
  [[nodiscard]] Scalar max_rod_error() const { return window_[1]; }
  [[nodiscard]] Scalar max_speed() const { return window_[2]; }
  [[nodiscard]] Scalar anchor_drift() const;

  static constexpr std::size_t kRodLength = 0;
  static constexpr std::size_t kPasses = 1;
  static constexpr int kWindowSteps = 60;

 private:
  void Measure();

  std::optional<PhysicsWorld> world_;
  std::array<SampleParameter, 2> parameters_{{
      {.name = "Rod length",
       .unit = "m",
       .min = 70,
       .max = 85,
       .step = 1,
       .scale = 10,
       .value = 80},
      {.name = "Passes",
       .unit = "",
       .min = 2,
       .max = 16,
       .step = 2,
       .scale = 1,
       .value = 8},
  }};
  std::array<SampleReadout, 4> readouts_{{
      {.name = "Hinge gap", .unit = "mm", .scale = 10},
      {.name = "Rod error", .unit = "mm", .scale = 10},
      {.name = "Max speed", .unit = "m/s", .scale = 1000},
      {.name = "Anchor drift", .unit = "mm", .scale = 10},
  }};
  BodyIndex axle_ = kInvalidBodyIndex;
  std::array<BodyIndex, 5> bodies_{};
  std::array<RevoluteJointIndex, 5> hinges_{};
  std::array<JointIndex, 2> rods_{};
  int step_ = 0;
  // The last full window and the current one: gap, rod error, speed
  std::array<Scalar, 3> window_{};
  std::array<Scalar, 3> current_{};
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_16_SEESAW_SOLUTIONS_OVER_CONSTRAINED_SAMPLE_H_
