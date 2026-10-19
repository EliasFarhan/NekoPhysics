#ifndef MAIN_STREAM_SAMPLE_STREAM_H_
#define MAIN_STREAM_SAMPLE_STREAM_H_

#include <cstdint>
#include <cstdio>

namespace neko::stream {

/// Steps of each sample in the default stream: 10 s at 60 Hz.
inline constexpr int kDefaultSteps = 600;

/// Writes the checksum stream of every sample and tutorial to @p out.
///
/// Runs each sample of the SampleRegistry and each entry of the tutorial
/// catalog for @p steps steps and writes one line per step,
/// "<id> <step> <checksum>" (step 0 is the state after Begin), after a header
/// "scalar <name> steps <steps>". The last line is "stream <hash>", an FNV-1a
/// hash of every checksum. Two builds that write the same text simulate the
/// same bits: compare the sha256 of the output on each platform.
///
/// A host without a console (the Switch) passes a file it opened itself.
///
/// @param steps The number of steps per sample, above 0.
/// @param out Where the lines go.
/// @return The stream hash, the value of the last line.
std::uint32_t RunSampleStream(int steps, std::FILE* out);

}  // namespace neko::stream

#endif  // MAIN_STREAM_SAMPLE_STREAM_H_
