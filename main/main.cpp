#include "samples/sample.h"

#include <SDL_main.h>
#include <SDL.h>
#include <SDL_render.h>
#include <imgui.h>
#include <imgui_impl_sdl2.h>
#include <imgui_impl_sdlrenderer2.h>

#ifdef TRACY_ENABLE
#include <tracy/Tracy.hpp>
#include <tracy/TracyC.h>
#endif

#include <iostream>


int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
	if (SDL_Init(SDL_INIT_VIDEO| SDL_INIT_GAMECONTROLLER)) {
		printf("error initializing SDL: %s\n", SDL_GetError());
	}

	SDL_Window* window = SDL_CreateWindow("Neko Physics Sample",
		SDL_WINDOWPOS_CENTERED,
		SDL_WINDOWPOS_CENTERED,
		1280, 720, SDL_WINDOW_RESIZABLE );



	SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
	
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGuiIO& io = ImGui::GetIO();
	(void)io;
	io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
	io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls

	// Setup Dear ImGui style
	ImGui::StyleColorsDark();
	ImGui_ImplSDL2_InitForSDLRenderer(window, renderer);
	ImGui_ImplSDLRenderer2_Init(renderer);

	neko::SampleManager sampleManager;


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
			ImGui_ImplSDL2_ProcessEvent(&event);
			switch (event.type)
		    {
			case SDL_QUIT:
				// handling of close button 
				isClosed = true;
				break;
			default:
				break;
			}
		}

		sampleManager.Update(dt);

		ImGui_ImplSDLRenderer2_NewFrame();
		ImGui_ImplSDL2_NewFrame();
		ImGui::NewFrame();

		
		sampleManager.DrawImGui();


		SDL_SetRenderDrawColor(renderer, 0, 0, 0, 0);
		SDL_RenderClear(renderer);

		sampleManager.Draw(renderer);

		ImGui::Render();
		ImGuiIO& io = ImGui::GetIO();
		SDL_RenderSetScale(renderer,
			io.DisplayFramebufferScale.x,
			io.DisplayFramebufferScale.y);
		//Update screen
		ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData(), renderer);
#ifdef TRACY_ENABLE
		FrameMark;
#endif
		lastTick = currentTick;


	}
	ImGui_ImplSDLRenderer2_Shutdown();
	ImGui_ImplSDL2_Shutdown();
	ImGui::DestroyContext();

	SDL_DestroyWindow(window);

	SDL_Quit();
    return 0;
}
