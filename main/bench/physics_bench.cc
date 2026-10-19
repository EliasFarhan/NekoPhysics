// Timings of PhysicsWorld::Step and PhysicsWorld::CopyFrom for the configured
// Scalar (N13 of the Splash Online plan), on the falling pile scene of the
// tests: 3 static boxes, 64 dynamic circles and boxes (some triggers) and a
// kinematic paddle, with contacts from the first step.
//
// Prints one line per measure, in microseconds:
// "<scalar> <measure> median <us> p99 <us> max <us>". Measures:
// - step: one Step of the pile, over the steps 1 to 600 (it falls, then
//   piles up and rests), repeated `rounds` times;
// - copy: one CopyFrom of the whole world at step 300 into another world;
// - rollback: one CopyFrom then 26 Steps, the worst case of a Splash rollback
//   (prediction window of 26 frames).
//
// Build in Release. Usage: physics_bench [rounds]   (default 20)
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <string_view>
#include <vector>

#include "neko/math/scalar_traits.h"
#include "neko/physics/physics.h"

#include "tests/scenes/falling_pile.h"

namespace {

using Clock = std::chrono::steady_clock;

constexpr int kSteps = 600;
constexpr int kRollbackFrames = 26;

double Microseconds(Clock::duration duration) {
  return std::chrono::duration<double, std::micro>(duration).count();
}

void Print(std::string_view measure, std::vector<double>& samples) {
  std::sort(samples.begin(), samples.end());
  const auto at = [&samples](double fraction) {
    const auto index = static_cast<std::size_t>(
        fraction * static_cast<double>(samples.size() - 1));
    return samples[index];
  };
  const std::string_view scalar =
      neko::ScalarTraits<neko::Scalar>::kDisplayName;
  std::printf("%-7.*s %-8.*s median %9.2f p99 %9.2f max %9.2f\n",
              static_cast<int>(scalar.size()), scalar.data(),
              static_cast<int>(measure.size()), measure.data(), at(0.5),
              at(0.99), samples.back());
}

}  // namespace

int main(int argc, char** argv) {
  int rounds = 20;
  if (argc > 1) {
    rounds = std::atoi(argv[1]);
    if (rounds <= 0) {
      std::fprintf(stderr, "usage: physics_bench [rounds > 0]\n");
      return 1;
    }
  }
  std::vector<double> step_samples;
  std::vector<double> copy_samples;
  std::vector<double> rollback_samples;
  std::uint32_t checksum = 0;
  neko::PhysicsWorld world;
  neko::PhysicsWorld copy;
  neko::PhysicsWorld at_300;
  for (int round = 0; round < rounds; ++round) {
    world.Clear();
    neko::scenes::BuildFallingPile(world);
    for (int step = 1; step <= kSteps; ++step) {
      const auto start = Clock::now();
      world.Step();
      step_samples.push_back(Microseconds(Clock::now() - start));
      if (step == 300) {
        at_300.CopyFrom(world);
      }
    }
    checksum ^= world.GenerateChecksum();
    for (int i = 0; i < 50; ++i) {
      const auto start = Clock::now();
      copy.CopyFrom(at_300);
      copy_samples.push_back(Microseconds(Clock::now() - start));
    }
    for (int i = 0; i < 10; ++i) {
      const auto start = Clock::now();
      copy.CopyFrom(at_300);
      for (int frame = 0; frame < kRollbackFrames; ++frame) {
        copy.Step();
      }
      rollback_samples.push_back(Microseconds(Clock::now() - start));
    }
  }
  Print("step", step_samples);
  Print("copy", copy_samples);
  Print("rollback", rollback_samples);
  // Keeps the work observable, and checks the rounds agree
  std::fprintf(
      stderr, "checksum %08x (%s)\n", checksum,
      rounds % 2 == 0 ? "0 when every round ends the same" : "one round's");
  return 0;
}
