#ifndef MAIN_SDL_SDL_HOST_H_
#define MAIN_SDL_SDL_HOST_H_

#include "main/samples/sample.h"

namespace neko {

struct SdlHostOptions {
  std::size_t initial_index = 0;
  // Optional chapter names, one per registry entry; empty keeps the flat menu.
  std::span<const char* const> groups{};
};
// PC host shared by the Sample executable and the tutorial programs: an SDL
// window with an ImGui menu (sample choice, restart, parameter sliders and
// readouts), a fixed-step accumulator calling SampleRegistry::FixedUpdate,
// and SdlSampleDrawer. Selects the requested sample, runs until the window closes
// and returns the process exit code. The executable's main.cc includes
// <SDL3/SDL_main.h>, which SDL needs in the file defining main.
int RunSdlHost(SampleRegistry& registry, const char* title,
               const SdlHostOptions& options = {});

}  // namespace neko

#endif  // MAIN_SDL_SDL_HOST_H_
