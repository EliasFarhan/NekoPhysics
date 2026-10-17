// Chapter 02, exercise "two ramps", on the PC host.
#include <memory>
#include <utility>
#include <vector>

#include <SDL3/SDL_main.h>

#include "main/samples/sample.h"
#include "main/sdl/sdl_host.h"
#include "tutorials/02_rotated_box/solutions/two_ramps_sample.h"

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv) {
  std::vector<neko::SampleRegistry::Entry> samples;
  samples.push_back(
      {"02 Two ramps", std::make_unique<neko::tutorial::TwoRampsSample>()});
  neko::SampleRegistry registry{std::move(samples)};
  return neko::RunSdlHost(registry, "NekoPhysics 02: Two ramps");
}
