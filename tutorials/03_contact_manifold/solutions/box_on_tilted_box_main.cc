// Chapter 03, exercise "box on a tilted box", on the PC host.
#include <memory>
#include <utility>
#include <vector>

#include <SDL3/SDL_main.h>

#include "main/samples/sample.h"
#include "main/sdl/sdl_host.h"
#include "tutorials/03_contact_manifold/solutions/box_on_tilted_box_sample.h"

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv) {
  std::vector<neko::SampleRegistry::Entry> samples;
  samples.push_back({"03 Box on a tilted box",
                     std::make_unique<neko::tutorial::BoxOnTiltedBoxSample>()});
  neko::SampleRegistry registry{std::move(samples)};
  return neko::RunSdlHost(registry, "NekoPhysics 03: Box on a tilted box");
}
