#include <cstdio>
#include <string_view>
#include <utility>
#include <vector>

#include <SDL3/SDL_main.h>

#include "main/sdl/sdl_host.h"
#include "tutorials/tutorial_catalog.h"

int main(int argc, char** argv) {
  const auto catalog = neko::tutorial::TutorialCatalog();
  std::size_t initial = 0;
  if (argc == 2 && std::string_view{argv[1]} == "--help") {
    std::puts("Usage: Tutorials [--list | --sample <id> | --help]");
    return 0;
  }
  if (argc == 2 && std::string_view{argv[1]} == "--list") {
    for (const auto& entry : catalog) {
      std::printf("%s\t%s (%s)\n", entry.id, entry.name,
                  entry.is_demo ? "demo" : "solution");
    }
    return 0;
  }
  if (argc == 3 && std::string_view{argv[1]} == "--sample") {
    initial = neko::tutorial::FindTutorial(argv[2]);
    if (initial == neko::SampleRegistry::kNone) {
      std::fprintf(stderr, "Unknown tutorial: %s. Use --list for IDs.\n",
                   argv[2]);
      return 1;
    }
  } else if (argc != 1) {
    std::fputs("Usage: Tutorials [--list | --sample <id> | --help]\n", stderr);
    return 1;
  }
  std::vector<neko::SampleRegistry::Entry> samples;
  std::vector<const char*> chapters;
  samples.reserve(catalog.size());
  chapters.reserve(catalog.size());
  for (const auto& entry : catalog) {
    samples.push_back({entry.name, entry.create()});
    chapters.push_back(entry.chapter);
  }
  neko::SampleRegistry registry{std::move(samples)};
  return neko::RunSdlHost(registry, "NekoPhysics Tutorials",
                          {.initial_index = initial, .groups = chapters});
}
