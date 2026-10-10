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

/// Reverses the velocity components that take a body further out of bounds.
///
/// Call it on every body after @ref PhysicsWorld::Step to keep a scene inside
/// the screen without walls or colliders (the kinematic, triggers and
/// collisions samples do). A component is flipped only when the body is
/// outside on that axis and still moving away, so a body that already turned
/// around is left alone and doesn't jitter on the border.
///
/// @param body The body to bounce; its `velocity` is modified.
/// @param bounds The rectangle to keep the body's position inside, usually
/// @ref SampleContext::world_bounds.
void BounceInside(Body& body, const Aabbf& bounds);

/// What the host platform gives a sample when it begins.
///
/// The host passes it to @ref SampleRegistry::Select, which forwards it to
/// @ref Sample::Begin. Both hosts use @ref sample_config::kWorldBounds.
struct SampleContext {
  /// World rectangle shown on screen, in meters with y pointing down.
  ///
  /// Samples place and bounce their bodies inside it, so the same scene fits
  /// the PC window and the DS top screen.
  Aabbf world_bounds{};
};

/// Physics scene shown by a host: the base class of every sample.
///
/// A sample knows nothing about the platform (no SDL, ImGui, fmt or libnds):
/// it simulates in @ref FixedUpdate and describes itself in a @ref DrawList.
/// The same code runs in the SDL window on PC (`main/sdl/main.cc`), in the DS
/// samples ROM (`nds/samples/main.cc`) and in `tests/samples_test.cc`, which
/// checks that every sample is deterministic.
///
/// Lifecycle, driven by @ref SampleRegistry, step by step:
///
/// - @ref Begin builds the scene when the sample is selected or restarted.
/// - @ref FixedUpdate advances it by exactly @ref kFixedDeltaTime. The PC host
///   calls it zero or more times per frame from an accumulator; the DS calls it
///   once per VBlank (60 Hz).
/// - @ref Draw fills the draw list once per rendered frame. It is `const`:
///   never change the simulation there, or the result would depend on the
///   frame rate.
/// - @ref End tears the scene down so that the next @ref Begin starts from a
///   clean world.
///
/// Rules that keep a sample deterministic (same checksum on every run, on PC
/// with `FIXED16` and on the DS, as rollback netcode requires):
///
/// - Build the scene from a @ref SampleRng with a fixed seed, and write
///   constants with `Ratio` instead of float literals.
/// - Read sizes and body counts from @ref sample_config, never hard-code a
///   screen size.
/// - Simulate only in @ref FixedUpdate, never with the frame time.
/// - Allocate in @ref Begin (or in the members' initializers), not per step.
///
/// How to add a new sample:
///
/// - Add `main/samples/07_my_sample.h` and `.cc` (they are picked up by the
///   `NekoPhysicsSamples` glob, so the PC host and the DS ROM both get them).
///   Derive from @ref Sample, and also from @ref ContactListener if you want
///   trigger or collision callbacks.
/// - Give the class a @ref PhysicsWorld member sized for the scene
///   (`max_bodies`, `max_colliders`...) with `dt = kFixedDeltaTime`.
///   Per-platform counts go in @ref sample_config; check that the
///   @ref DrawList capacities (@ref sample_config::kMaxDrawCircles...) still
///   fit.
/// - Register it in the @ref SampleRegistry constructor in
///   `main/samples/sample.cc` (`samples_.push_back`, and bump the
///   `reserve`), with the name shown in the menus. Both hosts
///   build their menus from the registry, so there is nothing else to change
///   in `main/sdl/` or `nds/samples/`.
/// - Update the sample count in `tests/samples_test.cc` (`HasSixSamples` and
///   the `INSTANTIATE_TEST_SUITE_P` range) so that the determinism tests run
///   on it too.
///
/// Skeleton:
/// @code
/// class MySample : public Sample {
///  public:
///   void Begin(const SampleContext& context) override {
///     bounds_ = context.world_bounds;
///     SampleRng rng{kSeed};
///     const BodyIndex index = world_.AddBody();
///     Body& body = world_.body(index);
///     body.position = context.world_bounds.GetCenter();
///     body.velocity = Vec2f{rng.Range(Scalar{-1}, Scalar{1}), Scalar{0}};
///     body.inverse_mass = Scalar{1};
///     color_ = rng.RandomColor();
///   }
///   void FixedUpdate() override {
///     world_.Step();
///     BounceInside(world_.body(BodyIndex{0}), bounds_);
///   }
///   void Draw(DrawList& draw_list) const override {
///     draw_list.circles.push_back(
///         {world_.body(BodyIndex{0}).position, Ratio(1, 10), color_});
///   }
///   void End() override { world_.Clear(); }
///   std::uint32_t GenerateChecksum() const override {
///     return world_.GenerateChecksum();
///   }
///
///  private:
///   static constexpr std::uint32_t kSeed = 0x07C0FFEEu;
///   PhysicsWorld world_{{.gravity = Vec2f::Zero(),
///                        .dt = kFixedDeltaTime,
///                        .max_bodies = 1,
///                        .max_colliders = 1,
///                        .max_touching_pairs = 1,
///                        .max_broadphase_pairs = 1}};
///   Aabbf bounds_{};
///   ColorId color_ = sample_color::kWhite;
/// };
/// @endcode
///
/// @see @ref SampleRegistry
/// @see @ref DrawList
class Sample {
 public:
  /// Destroys the sample.
  virtual ~Sample() = default;
  /// Builds the scene: adds the bodies and colliders and sets the listeners.
  ///
  /// Called each time the sample is selected or restarted, always after
  /// @ref End of the previous run, so it must produce the same initial state
  /// every time.
  ///
  /// @param context What the host gives the sample, such as the world bounds.
  virtual void Begin(const SampleContext& context) = 0;
  /// Advances the simulation by one @ref kFixedDeltaTime step.
  ///
  /// All simulation and game logic goes here: apply forces, call
  /// @ref PhysicsWorld::Step, react to the step. Never use wall-clock time.
  virtual void FixedUpdate() = 0;
  /// Describes the current state of the scene for the host to render.
  ///
  /// The registry has already cleared @p draw_list and set its camera; the
  /// sample only appends circles, boxes and lines in world space.
  ///
  /// @param draw_list The list to append the frame's shapes to.
  virtual void Draw(DrawList& draw_list) const = 0;
  /// Tears the scene down: clears the world, broadphase and per-body data.
  virtual void End() = 0;
  /// Returns a hash of the simulation state.
  ///
  /// Two runs of the same sample must give the same checksum after the same
  /// number of steps; `tests/samples_test.cc` checks it, and a rollback
  /// netcode would compare it between players.
  ///
  /// @return Usually @ref PhysicsWorld::GenerateChecksum of the sample's world.
  [[nodiscard]] virtual std::uint32_t GenerateChecksum() const = 0;

  /// Duration of one @ref FixedUpdate step, as a @ref Scalar: 1/60 s, one DS
  /// frame.
  static constexpr auto kFixedDeltaTime = Scalar{1.0f / 60.0f};
};

/// Owns every sample and runs the selected one.
///
/// The list of samples lives in the constructor (`main/samples/sample.cc`):
/// that is the only place where a new sample is registered. Each host builds
/// its own menu on top of it (ImGui combo box on PC, D-pad cursor on the DS)
/// and forwards its fixed steps and frames to it:
/// @code
/// SampleRegistry registry;
/// const SampleContext context{sample_config::kWorldBounds};
/// registry.Select(0, context);
/// DrawList draw_list;
/// draw_list.Reserve(sample_config::kMaxDrawCircles,
///                   sample_config::kMaxDrawBoxes,
///                   sample_config::kMaxDrawLines);
/// while (running) {
///   // as many times as kFixedDeltaTime fits in the elapsed time
///   registry.FixedUpdate();
///   registry.Draw(draw_list);
///   drawer.Render(draw_list);
/// }
/// @endcode
class SampleRegistry {
 public:
  /// Index meaning "no sample selected".
  static constexpr std::size_t kNone = std::numeric_limits<std::size_t>::max();

  /// Creates every sample, with none selected.
  SampleRegistry();
  /// Not copyable: it owns the samples.
  SampleRegistry(const SampleRegistry&) = delete;
  /// Not copyable: it owns the samples.
  SampleRegistry& operator=(const SampleRegistry&) = delete;
  /// Ends the current sample, then destroys every sample.
  ~SampleRegistry();

  /// Returns the number of registered samples.
  [[nodiscard]] std::size_t size() const { return samples_.size(); }
  /// Returns the menu name of a sample, such as `"01 Kinematic"`.
  ///
  /// @param index Sample index, less than @ref size.
  /// @return A string literal, valid for the registry's lifetime.
  [[nodiscard]] const char* name(std::size_t index) const {
    return samples_[index].name;
  }
  /// Returns the index of the selected sample.
  ///
  /// @return The index, or @ref kNone if no sample is selected.
  [[nodiscard]] std::size_t current_index() const { return current_index_; }
  /// Returns the selected sample, or `nullptr` if none is selected.
  [[nodiscard]] Sample* current() const {
    return current_index_ == kNone ? nullptr
                                   : samples_[current_index_].sample.get();
  }

  /// Ends the current sample and begins another one.
  ///
  /// Selecting the current index restarts it from its initial state.
  ///
  /// @param index Sample to begin; @ref kNone or any index not less than
  /// @ref size selects no sample.
  /// @param context Passed to @ref Sample::Begin and kept for @ref Draw.
  void Select(std::size_t index, const SampleContext& context);
  /// Advances the selected sample by one fixed step, if any.
  void FixedUpdate();
  /// Fills a draw list with the selected sample's frame.
  ///
  /// Clears the list, sets its camera to the context's world bounds, then
  /// lets the selected sample (if any) append its shapes.
  ///
  /// @param draw_list The list to fill; its capacity is kept.
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
