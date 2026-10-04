#ifndef MAIN_SAMPLES_SAMPLE_H_
#define MAIN_SAMPLES_SAMPLE_H_

#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <vector>

#include "neko/math/const.h"
#include "neko/math/shape.h"

#include "main/samples/draw_list.h"

namespace neko {

struct Body;

// Reverses the velocity components that take the body further out of bounds
void BounceInside(Body& body, const Aabbf& bounds);

// What the host platform gives a sample when it begins
struct SampleContext {
  // World rectangle shown on screen, y down: the samples bounce and place their
  // bodies inside it
  Aabbf world_bounds{};
};

// Physics scene shown by a host (SDL on PC, the DS ROM). Knows nothing about
// the platform: it simulates in FixedUpdate and describes itself in a DrawList.
class Sample {
 public:
  virtual ~Sample() = default;
  virtual void Begin(const SampleContext& context) = 0;
  // All simulation and game logic, once per kFixedDeltaTime
  virtual void FixedUpdate() = 0;
  virtual void Draw(DrawList& draw_list) const = 0;
  virtual void End() = 0;
  // Hash of the simulation state, to check that a sample is deterministic
  [[nodiscard]] virtual std::uint32_t GenerateChecksum() const = 0;

  static constexpr auto kFixedDeltaTime = Scalar{1.0f / 60.0f};
};

// The six samples and the selected one. Each host builds its own menu on top of
// it.
class SampleRegistry {
 public:
  static constexpr std::size_t kNone = std::numeric_limits<std::size_t>::max();

  SampleRegistry();
  SampleRegistry(const SampleRegistry&) = delete;
  SampleRegistry& operator=(const SampleRegistry&) = delete;
  ~SampleRegistry();

  [[nodiscard]] std::size_t size() const { return samples_.size(); }
  [[nodiscard]] const char* name(std::size_t index) const {
    return samples_[index].name;
  }
  [[nodiscard]] std::size_t current_index() const { return current_index_; }
  [[nodiscard]] Sample* current() const {
    return current_index_ == kNone ? nullptr
                                   : samples_[current_index_].sample.get();
  }

  // Ends the current sample and begins sample index (kNone: no sample).
  // Selecting the current index restarts it.
  void Select(std::size_t index, const SampleContext& context);
  void FixedUpdate();
  // Clears the list, sets its camera to the context's world bounds and lets the
  // current sample fill it
  void Draw(DrawList& draw_list) const;

 private:
  struct Entry {
    const char* name;
    std::unique_ptr<Sample> sample;
  };
  std::vector<Entry> samples_;
  std::size_t current_index_ = kNone;
  SampleContext context_{};
};

}  // namespace neko

#endif  // MAIN_SAMPLES_SAMPLE_H_
