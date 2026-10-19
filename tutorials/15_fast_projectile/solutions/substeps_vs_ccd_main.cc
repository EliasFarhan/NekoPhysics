// Chapter 15, exercise "substeps vs CCD", on the PC host.
#include <memory>
#include <utility>
#include <vector>

#include <SDL3/SDL_main.h>

#include "main/samples/sample.h"
#include "main/sdl/sdl_host.h"
#include "tutorials/15_fast_projectile/solutions/substeps_vs_ccd_sample.h"

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv) {
  std::vector<neko::SampleRegistry::Entry> samples;
  samples.push_back({"15 Substeps vs CCD",
                     std::make_unique<neko::tutorial::SubstepsVsCcdSample>()});
  neko::SampleRegistry registry{std::move(samples)};
  return neko::RunSdlHost(registry, "NekoPhysics 15: Substeps vs CCD");
}
