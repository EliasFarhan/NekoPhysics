#ifndef TUTORIALS_15_FAST_PROJECTILE_SOLUTIONS_TRIGGER_HITS_SAMPLE_H_
#define TUTORIALS_15_FAST_PROJECTILE_SOLUTIONS_TRIGGER_HITS_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "neko/physics/physics.h"

#include "main/samples/sample.h"

namespace neko::tutorial {

// Chapter 15, exercise "trigger hits": a fast trigger shot through a row of
// thin coins.
//
// Two water shots (trigger circles of radius 0.1 m) fly once to the right at
// the speed of the parameter "Speed" (80 m/s, 1.33 m per step), each along
// its own row of five coins: static trigger boxes 0.05 m wide at x = 3, 3.3,
// 3.45, 4.1 and 4.2 m from the left, added in a shuffled index order. The
// top shot is a bullet, the bottom one isn't.
//
// The bullet's sweep reports every coin its path crosses, several in one
// step when they are closer than a step, in the order of the path (by time
// of crossing, not by collider index), and the next step sends their exits.
// The plain shot collects a coin only when one of its steps happens to land
// on it: the overlap band is 2 r + 0.05 = 0.25 m wide, so at 1.33 m per step
// it collects about one coin in five.
//
// Readouts: the coins of each shot, whether the bullet's enters came in path
// order (1) and the most coins it entered in a single step. Drawn: the
// coins in yellow (grey once collected by their row's shot), the bullet in
// sky blue, the plain shot in cyan.
//
// The world is y-up, in meters and seconds; shots ignore gravity.
class TriggerHitsSample final : public Sample {
 public:
  void Begin(const SampleContext& context) override;
  void FixedUpdate() override;
  void Draw(DrawList& draw_list) const override;
  void End() override;
  [[nodiscard]] std::uint32_t GenerateChecksum() const override {
    return world_.GenerateChecksum();
  }
  [[nodiscard]] std::span<SampleParameter> parameters() override {
    return parameters_;
  }
  [[nodiscard]] std::span<const SampleReadout> readouts() const override {
    return readouts_;
  }

  // 0 for the bullet's row, 1 for the plain shot's
  [[nodiscard]] int coins(std::size_t row) const { return coins_[row]; }
  [[nodiscard]] bool in_path_order() const { return in_order_; }
  [[nodiscard]] int most_in_one_step() const { return most_in_one_step_; }
  // Coin exits of the bullet's row
  [[nodiscard]] int bullet_exits() const { return listener_.exits; }

  static constexpr std::size_t kSpeed = 0;
  static constexpr std::size_t kCoinCount = 5;
  static constexpr int kMeasuredSteps = 60;

 private:
  class CoinListener final : public ContactListener {
   public:
    void OnTriggerEnter(const ColliderPair& pair) override;
    void OnTriggerExit(const ColliderPair& pair) override;
    void OnCollisionEnter(const ColliderPair&) override {}
    void OnCollisionExit(const ColliderPair&) override {}
    TriggerHitsSample* sample = nullptr;
    int exits = 0;
  };

  void UpdateReadouts();

  PhysicsWorld world_{{.gravity = Vec2f{Scalar{0}, -Ratio(981, 100)},
                       .dt = kFixedDeltaTime,
                       .max_bodies = 16,
                       .max_colliders = 16,
                       .max_touching_pairs = 16,
                       .max_broadphase_pairs = 32,
                       .max_joints = 0,
                       .max_passing_pairs = 0}};
  std::array<SampleParameter, 1> parameters_{{
      {.name = "Speed",
       .unit = "m/s",
       .min = 5,
       .max = 100,
       .step = 5,
       .scale = 1,
       .value = 80},
  }};
  std::array<SampleReadout, 4> readouts_{{
      {.name = "Coins bullet", .unit = "", .scale = 1},
      {.name = "Coins plain", .unit = "", .scale = 1},
      {.name = "In path order", .unit = "", .scale = 1},
      {.name = "Most in a step", .unit = "", .scale = 1},
  }};

  CoinListener listener_;
  std::array<ColliderIndex, 2> shots_{};
  // coins_by_x_[row][k]: the k-th coin from the left of the row
  std::array<std::array<ColliderIndex, kCoinCount>, 2> coins_by_x_{};
  std::array<std::array<bool, kCoinCount>, 2> collected_{};
  std::array<int, 2> coins_{};
  // How far along its row the bullet's last coin is
  std::size_t next_in_order_ = 0;
  bool in_order_ = true;
  int coins_this_step_ = 0;
  int most_in_one_step_ = 0;
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_15_FAST_PROJECTILE_SOLUTIONS_TRIGGER_HITS_SAMPLE_H_
