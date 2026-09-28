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
#include <tracy/TracyC.h>
#endif

#include <cstdio>
#include <vector>

namespace
{

// Interim DrawList renderer at the old fixed 100 px/m, replaced by SdlSampleDrawer
void RenderDrawList(SDL_Renderer* renderer, const neko::DrawList& drawList)
{
    constexpr float pixelPerMeter = 100.0f;
    constexpr int circleResolution = 16;
    const auto toColor = [](neko::ColorId color)
    {
        const auto& c = neko::kSamplePalette[color];
        return SDL_FColor{ static_cast<float>(c.r) / 255.0f, static_cast<float>(c.g) / 255.0f, static_cast<float>(c.b) / 255.0f, 1.0f };
    };
    const auto toScreen = [&drawList](neko::Vec2f p)
    {
        return SDL_FPoint{ float{ p.x - drawList.camera.minBound.x } * pixelPerMeter,
            float{ p.y - drawList.camera.minBound.y } * pixelPerMeter };
    };
    static std::vector<SDL_Vertex> vertices;
    static std::vector<int> indices;
    vertices.clear();
    indices.clear();
    for (const auto& circle : drawList.circles)
    {
        const auto center = toScreen(circle.center);
        const float radius = float{ circle.radius } * pixelPerMeter;
        const auto color = toColor(circle.color);
        const int first = static_cast<int>(vertices.size());
        vertices.push_back({ center, color, {} });
        for (int j = 0; j < circleResolution; j++)
        {
            const float angle = 2.0f * neko::Pi<float>() * static_cast<float>(j) / float{ circleResolution };
            vertices.push_back({ { center.x + radius * std::cos(angle), center.y + radius * std::sin(angle) }, color, {} });
            indices.push_back(first);
            indices.push_back(first + 1 + j);
            indices.push_back(first + 1 + (j + 1) % circleResolution);
        }
    }
    if (!vertices.empty())
    {
        SDL_RenderGeometry(renderer, nullptr, vertices.data(), static_cast<int>(vertices.size()), indices.data(),
            static_cast<int>(indices.size()));
    }
    for (const auto& box : drawList.boxes)
    {
        const auto color = toColor(box.color);
        const auto topLeft = toScreen(box.center - box.halfSize);
        const SDL_FRect rect{ topLeft.x, topLeft.y, float{ box.halfSize.x } * 2.0f * pixelPerMeter,
            float{ box.halfSize.y } * 2.0f * pixelPerMeter };
        SDL_SetRenderDrawColorFloat(renderer, color.r, color.g, color.b, color.a);
        SDL_RenderFillRect(renderer, &rect);
    }
    for (const auto& line : drawList.lines)
    {
        const auto color = toColor(line.color);
        const auto a = toScreen(line.a);
        const auto b = toScreen(line.b);
        SDL_SetRenderDrawColorFloat(renderer, color.r, color.g, color.b, color.a);
        SDL_RenderLine(renderer, a.x, a.y, b.x, b.y);
    }
}

} // namespace


int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
	if (SDL_Init(SDL_INIT_VIDEO| SDL_INIT_GAMEPAD)) {
		printf("error initializing SDL: %s\n", SDL_GetError());
	}

	SDL_Window* window = SDL_CreateWindow("Neko Physics Sample",
		1280, 720,
		SDL_WINDOW_RESIZABLE );



	SDL_Renderer* renderer = SDL_CreateRenderer(window, nullptr);
	//SDL_GetRendererName(renderer);

	//fmt::println("Renderer: {}", info.name);
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGuiIO& io = ImGui::GetIO();
	(void)io;
	io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
	io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls

	// Setup Dear ImGui style
	ImGui::StyleColorsDark();
	ImGui_ImplSDL3_InitForSDLRenderer(window, renderer);
	ImGui_ImplSDLRenderer3_Init(renderer);

	neko::SampleRegistry sampleRegistry;
	const neko::SampleContext sampleContext{ neko::sample_config::WORLD_BOUNDS };
	neko::DrawList drawList;
	drawList.Reserve(neko::sample_config::MAX_DRAW_CIRCLES, neko::sample_config::MAX_DRAW_BOXES,
		neko::sample_config::MAX_DRAW_LINES);
	float fixedDt = 0.0f;


	auto lastTick = SDL_GetPerformanceCounter();

	bool isClosed = false;
	while (!isClosed) 
	{
		const auto currentTick = SDL_GetPerformanceCounter();
        const float dt = static_cast<float>(currentTick - lastTick) / static_cast<float>(SDL_GetPerformanceFrequency());
		SDL_Event event;
		// Events management 
		while (SDL_PollEvent(&event)) 
		{
			ImGui_ImplSDL3_ProcessEvent(&event);
			switch (event.type)
		    {
			case SDL_EVENT_QUIT:
				// handling of close button 
				isClosed = true;
				break;
			default:
				break;
			}
		}

		fixedDt += dt;
		while (fixedDt > float{ neko::Sample::fixedDeltaTime })
		{
			sampleRegistry.FixedUpdate();
			fixedDt -= float{ neko::Sample::fixedDeltaTime };
		}

		ImGui_ImplSDLRenderer3_NewFrame();
		ImGui_ImplSDL3_NewFrame();
		ImGui::NewFrame();

		
		ImGui::Begin("Sample Manager");
		const auto current = sampleRegistry.currentIndex();
		if (ImGui::BeginCombo("Samples", current == neko::SampleRegistry::NONE ? "None" : sampleRegistry.name(current)))
		{
			if (ImGui::Selectable("None", current == neko::SampleRegistry::NONE))
			{
				sampleRegistry.Select(neko::SampleRegistry::NONE, sampleContext);
			}
			for (std::size_t index = 0; index < sampleRegistry.size(); index++)
			{
				if (ImGui::Selectable(sampleRegistry.name(index), current == index))
				{
					sampleRegistry.Select(index, sampleContext);
				}
			}
			ImGui::EndCombo();
		}
		ImGui::End();


		SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
		SDL_RenderClear(renderer);

		sampleRegistry.Draw(drawList);
		RenderDrawList(renderer, drawList);

		ImGui::Render();
		/*SDL_RenderSetScale(renderer,
			io.DisplayFramebufferScale.x,
			io.DisplayFramebufferScale.y);
		 */
		//Update screen
		ImGui_ImplSDLRenderer3_RenderDrawData(ImGui::GetDrawData(), renderer);
		SDL_RenderPresent(renderer);

		lastTick = currentTick;
#ifdef TRACY_ENABLE
		FrameMark;
#endif

	}
	ImGui_ImplSDLRenderer3_Shutdown();
	ImGui_ImplSDL3_Shutdown();
	ImGui::DestroyContext();

	SDL_DestroyWindow(window);

	SDL_Quit();
    return 0;
}
