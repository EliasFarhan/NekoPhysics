#include "main/sdl/sdl_host.h"

#include <imgui.h>

#include <SDL3/SDL.h>
#include <SDL3/SDL_render.h>
#include <imgui_impl_sdl3.h>
#include <imgui_impl_sdlrenderer3.h>

#include "main/samples/sample_config.h"
#include "main/sdl/sdl_sample_drawer.h"

#ifdef TRACY_ENABLE
#include <tracy/Tracy.hpp>
#endif

#include <array>
#include <cstdio>

namespace {

// Longest frame time fed to the accumulator: after a stall (a breakpoint, a
// window drag) the simulation slows down instead of running hundreds of steps
// to catch up
constexpr float kMaxFrameTime = 0.25f;

void DrawSampleMenu(neko::SampleRegistry& registry,
                    const neko::SampleContext& context) {
  // Wide enough for the parameter sliders and their labels
  ImGui::SetNextWindowSize(ImVec2{360.0f, 0.0f}, ImGuiCond_FirstUseEver);
  ImGui::Begin("Sample Manager");
  const auto current = registry.current_index();
  if (ImGui::BeginCombo("Samples", current == neko::SampleRegistry::kNone
                                       ? "None"
                                       : registry.name(current))) {
    if (ImGui::Selectable("None", current == neko::SampleRegistry::kNone)) {
      registry.Select(neko::SampleRegistry::kNone, context);
    }
    for (std::size_t index = 0; index < registry.size(); index++) {
      if (ImGui::Selectable(registry.name(index), current == index)) {
        registry.Select(index, context);
      }
    }
    ImGui::EndCombo();
  }
  if (current != neko::SampleRegistry::kNone && ImGui::Button("Restart")) {
    registry.Select(current, context);
  }
  // One slider per parameter, over the step count: the sample restarts when a
  // value changes
  const auto parameters = registry.parameters();
  for (std::size_t index = 0; index < parameters.size(); ++index) {
    const auto& parameter = parameters[index];
    const int step = parameter.step > 0 ? parameter.step : 1;
    int steps = (parameter.value - parameter.min) / step;
    // The slider shows the formatted value, so the format holds no %d
    std::array<char, 32> text{};
    const auto length =
        neko::FormatDecimal(parameter.value, parameter.scale,
                            std::span{text.data(), text.size() - 1});
    std::snprintf(text.data() + length, text.size() - length, " %s",
                  parameter.unit);
    if (ImGui::SliderInt(parameter.name, &steps, 0,
                         (parameter.max - parameter.min) / step, text.data(),
                         ImGuiSliderFlags_NoInput)) {
      registry.SetParameter(index, parameter.min + steps * step);
    }
  }
  for (const auto& readout : registry.readouts()) {
    if (!readout.is_valid) {
      ImGui::Text("%s: -", readout.name);
      continue;
    }
    std::array<char, 32> text{};
    const auto length = neko::FormatDecimal(
        readout.value, readout.scale, std::span{text.data(), text.size() - 1});
    text[length] = '\0';
    ImGui::Text("%s: %s %s", readout.name, text.data(), readout.unit);
  }
  ImGui::End();
}

}  // namespace

int neko::RunSdlHost(SampleRegistry& registry, const char* title) {
  if (!SDL_Init(SDL_INIT_VIDEO | SDL_INIT_GAMEPAD)) {
    std::printf("error initializing SDL: %s\n", SDL_GetError());
    return 1;
  }

  SDL_Window* window = SDL_CreateWindow(title, 1280, 720, SDL_WINDOW_RESIZABLE);
  SDL_Renderer* renderer = SDL_CreateRenderer(window, nullptr);

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO& io = ImGui::GetIO();
  io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
  io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;
  ImGui::StyleColorsDark();
  ImGui_ImplSDL3_InitForSDLRenderer(window, renderer);
  ImGui_ImplSDLRenderer3_Init(renderer);

  {
    const neko::SampleContext context{neko::sample_config::kWorldBounds};
    registry.Select(0, context);
    neko::DrawList draw_list;
    draw_list.Reserve(neko::sample_config::kMaxDrawCircles,
                      neko::sample_config::kMaxDrawBoxes,
                      neko::sample_config::kMaxDrawLines);
    neko::SdlSampleDrawer drawer{renderer};

    const float fixed_delta_time = float{neko::Sample::kFixedDeltaTime};
    float accumulator = 0.0f;
    auto last_tick = SDL_GetPerformanceCounter();
    bool is_closed = false;
    while (!is_closed) {
      const auto current_tick = SDL_GetPerformanceCounter();
      const float frame_time =
          static_cast<float>(current_tick - last_tick) /
          static_cast<float>(SDL_GetPerformanceFrequency());
      last_tick = current_tick;

      SDL_Event event;
      while (SDL_PollEvent(&event)) {
        ImGui_ImplSDL3_ProcessEvent(&event);
        if (event.type == SDL_EVENT_QUIT) {
          is_closed = true;
        }
      }

      accumulator += frame_time < kMaxFrameTime ? frame_time : kMaxFrameTime;
      while (accumulator >= fixed_delta_time) {
        registry.FixedUpdate();
        accumulator -= fixed_delta_time;
      }

      ImGui_ImplSDLRenderer3_NewFrame();
      ImGui_ImplSDL3_NewFrame();
      ImGui::NewFrame();
      DrawSampleMenu(registry, context);

      SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
      SDL_RenderClear(renderer);
      registry.Draw(draw_list);
      drawer.Render(draw_list);

      ImGui::Render();
      ImGui_ImplSDLRenderer3_RenderDrawData(ImGui::GetDrawData(), renderer);
      SDL_RenderPresent(renderer);
#ifdef TRACY_ENABLE
      FrameMark;
#endif
    }
  }

  ImGui_ImplSDLRenderer3_Shutdown();
  ImGui_ImplSDL3_Shutdown();
  ImGui::DestroyContext();
  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();
  return 0;
}
