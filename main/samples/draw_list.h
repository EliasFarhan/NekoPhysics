#ifndef MAIN_SAMPLES_DRAW_LIST_H_
#define MAIN_SAMPLES_DRAW_LIST_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "neko/math/shape.h"
#include "neko/math/vec2.h"

namespace neko {

// Index into kSamplePalette. 0 is transparent (the DS sprite and bitmap
// palettes reserve entry 0).
using ColorId = std::uint8_t;

struct PaletteColor {
  std::uint8_t r = 0;
  std::uint8_t g = 0;
  std::uint8_t b = 0;
};

// Colors shared by every backend: on the DS, ColorId i is sprite palette bank i
// (fill = entry 1) and bitmap BG palette entry i, so no backend has to quantize
// anything
inline constexpr std::array<PaletteColor, 16> kSamplePalette = {{
    {0, 0, 0},        // transparent
    {255, 255, 255},  // white
    {160, 160, 160},  // grey
    {80, 80, 80},     // dark grey
    {230, 60, 60},    // red
    {60, 200, 80},    // green
    {60, 110, 230},   // blue
    {240, 220, 60},   // yellow
    {60, 210, 220},   // cyan
    {220, 70, 200},   // magenta
    {245, 150, 50},   // orange
    {150, 90, 230},   // purple
    {245, 150, 190},  // pink
    {170, 230, 70},   // lime
    {40, 160, 140},   // teal
    {130, 190, 250},  // sky
}};

namespace sample_color {
// Not TRANSPARENT: wingdi.h defines it as a macro
inline constexpr ColorId kEmpty = 0;
inline constexpr ColorId kWhite = 1;
inline constexpr ColorId kGrey = 2;
inline constexpr ColorId kDarkGrey = 3;
inline constexpr ColorId kRed = 4;
inline constexpr ColorId kGreen = 5;
inline constexpr ColorId kBlue = 6;
inline constexpr ColorId kYellow = 7;
inline constexpr ColorId kCyan = 8;
inline constexpr ColorId kMagenta = 9;
inline constexpr ColorId kOrange = 10;
inline constexpr ColorId kPurple = 11;
inline constexpr ColorId kPink = 12;
inline constexpr ColorId kLime = 13;
inline constexpr ColorId kTeal = 14;
inline constexpr ColorId kSky = 15;
// Non-grey entries, the range random colors are picked from
inline constexpr ColorId kFirstBright = kRed;
inline constexpr ColorId kLastBright = kSky;
}  // namespace sample_color

struct DrawCircle {
  Vec2f center{};
  Scalar radius{};
  ColorId color = sample_color::kWhite;
};

struct DrawBox {
  Vec2f center{};
  Vec2f half_size{};
  ColorId color = sample_color::kWhite;
};

struct DrawLine {
  Vec2f a{};
  Vec2f b{};
  ColorId color = sample_color::kWhite;
};

// What a sample shows for one frame, in world space (meters, y down). Backends
// map it to the screen and draw circles, then boxes, then lines.
struct DrawList {
  // World rectangle to show: the SampleContext bounds
  Aabbf camera{};
  std::vector<DrawCircle> circles;
  std::vector<DrawBox> boxes;
  std::vector<DrawLine> lines;

  // Called once by the host: Clear keeps the capacity, so frames don't allocate
  void Reserve(std::size_t circle_count, std::size_t box_count,
               std::size_t line_count) {
    circles.reserve(circle_count);
    boxes.reserve(box_count);
    lines.reserve(line_count);
  }

  void Clear() {
    circles.clear();
    boxes.clear();
    lines.clear();
  }

  [[nodiscard]] bool empty() const {
    return circles.empty() && boxes.empty() && lines.empty();
  }
};

class SampleDrawer {
 public:
  virtual ~SampleDrawer() = default;
  virtual void Render(const DrawList& draw_list) = 0;
};

}  // namespace neko

#endif  // MAIN_SAMPLES_DRAW_LIST_H_
