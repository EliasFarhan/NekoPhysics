// Chapter 15 program: the same shot with and without continuous collision,
// against thin walls and moving players, on the PC host.
#include <memory>
#include <utility>
#include <vector>

#include <SDL3/SDL_main.h>

#include "main/samples/sample.h"
#include "main/sdl/sdl_host.h"
#include "tutorials/15_fast_projectile/fast_projectile_sample.h"

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv) {
  std::vector<neko::SampleRegistry::Entry> samples;
  samples.push_back({"15 Fast projectiles",
                     std::make_unique<neko::tutorial::FastProjectileSample>()});
  neko::SampleRegistry registry{std::move(samples)};
  return neko::RunSdlHost(registry, "NekoPhysics 15: Fast projectiles");
}
