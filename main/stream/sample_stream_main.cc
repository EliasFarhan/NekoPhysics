// Checksum stream of every sample and tutorial on stdout, for cross-platform
// checks (see neko::stream::RunSampleStream).
//
// Usage: sample_stream [steps]   (default 600, 10 s at 60 Hz)
#include <cstdio>
#include <cstdlib>

#include "main/stream/sample_stream.h"

int main(int argc, char** argv) {
  int steps = neko::stream::kDefaultSteps;
  if (argc > 1) {
    steps = std::atoi(argv[1]);
    if (steps <= 0) {
      std::fprintf(stderr, "usage: sample_stream [steps > 0]\n");
      return 1;
    }
  }
  const auto hash = neko::stream::RunSampleStream(steps, stdout);
  std::fprintf(stderr, "%d steps per sample, stream %08x\n", steps, hash);
  return 0;
}
