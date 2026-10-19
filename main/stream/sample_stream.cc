#include "main/stream/sample_stream.h"

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <string_view>
#include <utility>
#include <vector>

#include "neko/math/scalar_traits.h"
#include "neko/physics/physics.h"

#include "main/samples/sample.h"
#include "main/samples/sample_config.h"
#include "tutorials/tutorial_catalog.h"

namespace {

class StreamHash {
 public:
  void Add(std::uint32_t word) {
    for (int shift = 0; shift < 32; shift += 8) {
      hash_ ^= (word >> shift) & 0xFFu;
      hash_ *= 16777619u;
    }
  }
  [[nodiscard]] std::uint32_t Get() const { return hash_; }

 private:
  std::uint32_t hash_ = 2166136261u;
};

// Runs sample `index` of `registry` the way the hosts do (Select, then
// FixedUpdate), one line per step
void Run(neko::SampleRegistry& registry, std::size_t index, int steps,
         StreamHash& hash, std::FILE* out) {
  const neko::SampleContext context{neko::sample_config::kWorldBounds};
  const std::string_view id = registry.name(index);
  registry.Select(index, context);
  const neko::Sample* sample = registry.current();
  if (sample == nullptr) {
    return;
  }
  for (int step = 0; step <= steps; ++step) {
    if (step > 0) {
      registry.FixedUpdate();
    }
    const std::uint32_t checksum = sample->GenerateChecksum();
    hash.Add(checksum);
    std::fprintf(out, "%.*s %d %08x\n", static_cast<int>(id.size()), id.data(),
                 step, checksum);
  }
  registry.Select(neko::SampleRegistry::kNone, context);
}

}  // namespace

namespace neko::stream {

std::uint32_t RunSampleStream(int steps, std::FILE* out) {
  const std::string_view scalar =
      neko::ScalarTraits<neko::Scalar>::kDisplayName;
  std::fprintf(out, "scalar %.*s steps %d\n", static_cast<int>(scalar.size()),
               scalar.data(), steps);
  StreamHash hash;
  neko::SampleRegistry registry;
  for (std::size_t i = 0; i < registry.size(); ++i) {
    Run(registry, i, steps, hash, out);
  }
  for (const auto& entry : neko::tutorial::TutorialCatalog()) {
    std::vector<neko::SampleRegistry::Entry> one;
    one.push_back({entry.id, entry.create()});
    neko::SampleRegistry tutorial{std::move(one)};
    Run(tutorial, 0, steps, hash, out);
  }
  std::fprintf(out, "stream %08x\n", hash.Get());
  return hash.Get();
}

}  // namespace neko::stream
