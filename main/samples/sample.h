#ifndef MAIN_SAMPLES_SAMPLE_H_
#define MAIN_SAMPLES_SAMPLE_H_

#include "main/samples/draw_list.h"
#include "neko/math/const.h"
#include "neko/math/shape.h"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <vector>

namespace neko
{

struct Body;

/**
 * @brief Reverses the velocity components that take the body further out of bounds
 */
void BounceInside(Body& body, const Aabbf& bounds);

/**
 * @brief What the host platform gives a sample when it begins
 */
struct SampleContext
{
    /**
     * @brief World rectangle shown on screen, y down: the samples bounce and place their bodies inside it
     */
    Aabbf worldBounds{};
};

/**
 * @brief Physics scene shown by a host (SDL on PC, the DS ROM). Knows nothing about the platform: it simulates in
 * FixedUpdate and describes itself in a DrawList.
 */
class Sample
{
public:
    virtual ~Sample() = default;
    virtual void Begin(const SampleContext& context) = 0;
    /**
     * @brief All simulation and game logic, once per fixedDeltaTime
     */
    virtual void FixedUpdate() = 0;
    virtual void Draw(DrawList& drawList) const = 0;
    virtual void End() = 0;
    /**
     * @brief Hash of the simulation state, to check that a sample is deterministic
     */
    [[nodiscard]] virtual std::uint32_t GenerateChecksum() const = 0;

    static constexpr auto fixedDeltaTime = Scalar{ 1.0f / 60.0f };
};

/**
 * @brief The six samples and the selected one. Each host builds its own menu on top of it.
 */
class SampleRegistry
{
public:
    static constexpr std::size_t NONE = std::numeric_limits<std::size_t>::max();

    SampleRegistry();
    SampleRegistry(const SampleRegistry&) = delete;
    SampleRegistry& operator=(const SampleRegistry&) = delete;
    ~SampleRegistry();

    [[nodiscard]] std::size_t size() const { return samples_.size(); }
    [[nodiscard]] const char* name(std::size_t index) const { return samples_[index].name; }
    [[nodiscard]] std::size_t currentIndex() const { return currentIndex_; }
    [[nodiscard]] Sample* current() const
    {
        return currentIndex_ == NONE ? nullptr : samples_[currentIndex_].sample.get();
    }

    /**
     * @brief Ends the current sample and begins sample index (NONE: no sample). Selecting the current index restarts it.
     */
    void Select(std::size_t index, const SampleContext& context);
    void FixedUpdate();
    /**
     * @brief Clears the list, sets its camera to the context's world bounds and lets the current sample fill it
     */
    void Draw(DrawList& drawList) const;

private:
    struct Entry
    {
        const char* name;
        std::unique_ptr<Sample> sample;
    };
    std::vector<Entry> samples_;
    std::size_t currentIndex_ = NONE;
    SampleContext context_{};
};

} // namespace neko

#endif  // MAIN_SAMPLES_SAMPLE_H_
