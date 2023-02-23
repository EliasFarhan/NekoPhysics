#include "samples/sample.h"

#include <SDL_main.h>
#include <SDL.h>
#include <SDL_render.h>
#include <imgui.h>
#include <imgui_impl_sdl2.h>
#include <imgui_impl_opengl3.h>

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

	SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);

	SDL_Window* window = SDL_CreateWindow("Neko Physics Sample",
		SDL_WINDOWPOS_CENTERED,
		SDL_WINDOWPOS_CENTERED,
		1280, 720, SDL_WINDOW_RESIZABLE );

    const auto driverCount =  SDL_GetNumRenderDrivers();
	int driverIndex = -1;
    for(int i = 0; i < driverCount; i++)
    {
        SDL_RendererInfo info{};
        if(SDL_GetRenderDriverInfo(i, &info))
        {
            std::cerr << "Error loading driver info for index: "<<i<<'\n';
        }
        std::cout << "Driver Info "<< i + 1 <<": "<<info.name<<'\n';
		std::string driverName = info.name;
		if(driverName.find("opengl") != std::string::npos && driverName.find("opengles") == std::string::npos)
		{
			std::cout << "Selected OpenGL renderer\n";
			driverIndex = i;
		}
    }

	const auto glRenderContext = SDL_GL_CreateContext(window);
	SDL_SetHint(SDL_HINT_RENDER_VSYNC, "1");
	SDL_Renderer* renderer = SDL_CreateRenderer(window, driverIndex, SDL_RENDERER_ACCELERATED);
	
	// setup Dear ImGui context
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();

	ImGuiIO& io = ImGui::GetIO();
	(void)io;
	io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;  // Enable Keyboard Controls
	io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;  // Enable Keyboard Gamepad
    ImGui::StyleColorsClassic();
	ImGui_ImplSDL2_InitForOpenGL(window, glRenderContext);
	ImGui_ImplOpenGL3_Init("#version 330");

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

		ImGui_ImplOpenGL3_NewFrame();
		ImGui_ImplSDL2_NewFrame();
		ImGui::NewFrame();

		
		sampleManager.DrawImGui();


		SDL_SetRenderDrawColor(renderer, 0, 0, 0, 0);
		SDL_RenderClear(renderer);

		sampleManager.Draw(renderer);

		ImGui::Render();
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
		SDL_RenderPresent(renderer);
#ifdef TRACY_ENABLE
		FrameMark;
#endif
		lastTick = currentTick;


	}
	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplSDL2_Shutdown();

	SDL_DestroyWindow(window);

	SDL_Quit();
    return 0;
}
