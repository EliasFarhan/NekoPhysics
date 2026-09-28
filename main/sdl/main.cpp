// PC sample host: SDL window, ImGui sample menu, fixed-step accumulator. The samples themselves are platform-neutral
// (NekoPhysicsSamples) and are rendered through SdlSampleDrawer.
#include "sdl_sample_drawer.h"

#include "samples/sample.h"
#include "samples/sample_config.h"

#include <SDL3/SDL_main.h>
#include <SDL3/SDL.h>
#include <SDL3/SDL_render.h>
#include <imgui.h>
#include <imgui_impl_sdl3.h>
#include <imgui_impl_sdlrenderer3.h>

#ifdef TRACY_ENABLE
#include <tracy/Tracy.hpp>
#endif

#include <cstdio>

namespace
{

/**
 * @brief Longest frame time fed to the accumulator: after a stall (a breakpoint, a window drag) the simulation slows
 * down instead of running hundreds of steps to catch up
 */
constexpr float MAX_FRAME_TIME = 0.25f;

void DrawSampleMenu(neko::SampleRegistry& registry, const neko::SampleContext& context)
{
    ImGui::Begin("Sample Manager");
    const auto current = registry.currentIndex();
    if (ImGui::BeginCombo("Samples", current == neko::SampleRegistry::NONE ? "None" : registry.name(current)))
    {
        if (ImGui::Selectable("None", current == neko::SampleRegistry::NONE))
        {
            registry.Select(neko::SampleRegistry::NONE, context);
        }
        for (std::size_t index = 0; index < registry.size(); index++)
        {
            if (ImGui::Selectable(registry.name(index), current == index))
            {
                registry.Select(index, context);
            }
        }
        ImGui::EndCombo();
    }
    if (current != neko::SampleRegistry::NONE && ImGui::Button("Restart"))
    {
        registry.Select(current, context);
    }
    ImGui::End();
}

} // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    if (!SDL_Init(SDL_INIT_VIDEO | SDL_INIT_GAMEPAD))
    {
        std::printf("error initializing SDL: %s\n", SDL_GetError());
        return 1;
    }

    SDL_Window* window = SDL_CreateWindow("Neko Physics Sample", 1280, 720, SDL_WINDOW_RESIZABLE);
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
        neko::SampleRegistry registry;
        const neko::SampleContext context{ neko::sample_config::WORLD_BOUNDS };
        registry.Select(0, context);
        neko::DrawList drawList;
        drawList.Reserve(neko::sample_config::MAX_DRAW_CIRCLES, neko::sample_config::MAX_DRAW_BOXES,
            neko::sample_config::MAX_DRAW_LINES);
        neko::SdlSampleDrawer drawer{ renderer };

        const float fixedDeltaTime = float{ neko::Sample::fixedDeltaTime };
        float accumulator = 0.0f;
        auto lastTick = SDL_GetPerformanceCounter();
        bool isClosed = false;
        while (!isClosed)
        {
            const auto currentTick = SDL_GetPerformanceCounter();
            const float frameTime = static_cast<float>(currentTick - lastTick) /
                static_cast<float>(SDL_GetPerformanceFrequency());
            lastTick = currentTick;

            SDL_Event event;
            while (SDL_PollEvent(&event))
            {
                ImGui_ImplSDL3_ProcessEvent(&event);
                if (event.type == SDL_EVENT_QUIT)
                {
                    isClosed = true;
                }
            }

            accumulator += frameTime < MAX_FRAME_TIME ? frameTime : MAX_FRAME_TIME;
            while (accumulator >= fixedDeltaTime)
            {
                registry.FixedUpdate();
                accumulator -= fixedDeltaTime;
            }

            ImGui_ImplSDLRenderer3_NewFrame();
            ImGui_ImplSDL3_NewFrame();
            ImGui::NewFrame();
            DrawSampleMenu(registry, context);

            SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
            SDL_RenderClear(renderer);
            registry.Draw(drawList);
            drawer.Render(drawList);

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
