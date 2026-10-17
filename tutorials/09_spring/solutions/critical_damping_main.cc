// Chapter 09, exercise "critical damping", on the PC host.
#include <memory>
#include <utility>
#include <vector>

#include <SDL3/SDL_main.h>

#include "main/samples/sample.h"
#include "main/sdl/sdl_host.h"
#include "tutorials/09_spring/solutions/critical_damping_sample.h"

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv) {
  std::vector<neko::SampleRegistry::Entry> samples;
  samples.push_back(
      {"09 Critical damping",
       std::make_unique<neko::tutorial::CriticalDampingSample>()});
  neko::SampleRegistry registry{std::move(samples)};
  return neko::RunSdlHost(registry, "NekoPhysics 09: Critical damping");
}
