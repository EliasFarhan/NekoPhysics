// Chapter 14 program: the patrol lift, the lagging platform, the rider carry
// and the wrapping TicTac platform on the PC host.
#include <memory>
#include <utility>
#include <vector>

#include <SDL3/SDL_main.h>

#include "main/samples/sample.h"
#include "main/sdl/sdl_host.h"
#include "tutorials/14_moving_platform/moving_platform_sample.h"

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv) {
  std::vector<neko::SampleRegistry::Entry> samples;
  samples.push_back({"14 Moving platforms",
                     std::make_unique<neko::tutorial::MovingPlatformSample>()});
  neko::SampleRegistry registry{std::move(samples)};
  return neko::RunSdlHost(registry, "NekoPhysics 14: Moving platforms");
}
