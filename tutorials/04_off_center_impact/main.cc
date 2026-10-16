// Chapter 04 program: the off-center impact experiment on the PC host.
#include <memory>
#include <utility>
#include <vector>

#include <SDL3/SDL_main.h>

#include "main/samples/sample.h"
#include "main/sdl/sdl_host.h"
#include "tutorials/04_off_center_impact/off_center_impact_sample.h"

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv) {
  std::vector<neko::SampleRegistry::Entry> samples;
  samples.push_back(
      {"04 Off-center impact",
       std::make_unique<neko::tutorial::OffCenterImpactSample>()});
  neko::SampleRegistry registry{std::move(samples)};
  return neko::RunSdlHost(registry, "NekoPhysics 04: Off-center impact");
}
