// PC sample host: the six NekoPhysicsSamples in an SDL window (see
// main/sdl/sdl_host.h). The samples themselves are platform-neutral and are
// rendered through SdlSampleDrawer.
#include <SDL3/SDL_main.h>

#include "main/samples/sample.h"
#include "main/sdl/sdl_host.h"

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv) {
  neko::SampleRegistry registry;
  return neko::RunSdlHost(registry, "Neko Physics Sample");
}
