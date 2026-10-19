#ifndef TUTORIALS_TUTORIAL_CATALOG_H_
#define TUTORIALS_TUTORIAL_CATALOG_H_

#include <memory>
#include <span>
#include <string_view>

#include "main/samples/sample.h"

namespace neko::tutorial {
/// Stable launch ID and chapter metadata for one tutorial experiment.
struct TutorialEntry {
  const char* id;
  const char* chapter;
  const char* name;
  bool is_demo;
  std::unique_ptr<Sample> (*create)();
};
/// All chapter demos and exercise solutions, in chapter order.
std::span<const TutorialEntry> TutorialCatalog();
/// Returns SampleRegistry::kNone when the launch ID is unknown.
std::size_t FindTutorial(std::string_view id);
}  // namespace neko::tutorial

#endif  // TUTORIALS_TUTORIAL_CATALOG_H_
