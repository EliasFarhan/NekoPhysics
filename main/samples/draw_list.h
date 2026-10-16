#ifndef MAIN_SAMPLES_DRAW_LIST_H_
#define MAIN_SAMPLES_DRAW_LIST_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "neko/math/shape.h"
#include "neko/math/vec2.h"

namespace neko {

/// Index into @ref kSamplePalette.
///
/// 0 (@ref sample_color::kEmpty) is transparent: the DS sprite and bitmap
/// palettes reserve entry 0. Use the @ref sample_color constants rather than
/// raw numbers.
using ColorId = std::uint8_t;

/// 8-bit RGB color of a @ref kSamplePalette entry.
struct PaletteColor {
  std::uint8_t r = 0;  ///< Red, 0 to 255.
  std::uint8_t g = 0;  ///< Green, 0 to 255.
  std::uint8_t b = 0;  ///< Blue, 0 to 255.
};

/// The 16 colors (@ref PaletteColor) shared by every backend, indexed by
/// @ref ColorId.
///
/// Samples only pick colors from this fixed palette, so no backend has to
/// quantize anything: on the DS, @ref ColorId `i` is sprite palette bank `i`
/// (fill = entry 1) and bitmap BG palette entry `i`; the SDL drawer reads the
/// RGB values directly.
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

/// Named @ref ColorId values, one per @ref kSamplePalette entry.
namespace sample_color {
/// Transparent: draws nothing.
///
/// Not called TRANSPARENT because wingdi.h defines that as a macro.
inline constexpr ColorId kEmpty = 0;
inline constexpr ColorId kWhite = 1;     ///< White.
inline constexpr ColorId kGrey = 2;      ///< Grey.
inline constexpr ColorId kDarkGrey = 3;  ///< Dark grey.
inline constexpr ColorId kRed = 4;       ///< Red.
inline constexpr ColorId kGreen = 5;     ///< Green.
inline constexpr ColorId kBlue = 6;      ///< Blue.
inline constexpr ColorId kYellow = 7;    ///< Yellow.
inline constexpr ColorId kCyan = 8;      ///< Cyan.
inline constexpr ColorId kMagenta = 9;   ///< Magenta.
inline constexpr ColorId kOrange = 10;   ///< Orange.
inline constexpr ColorId kPurple = 11;   ///< Purple.
inline constexpr ColorId kPink = 12;     ///< Pink.
inline constexpr ColorId kLime = 13;     ///< Lime.
inline constexpr ColorId kTeal = 14;     ///< Teal.
inline constexpr ColorId kSky = 15;      ///< Sky blue.
/// First non-grey color, the lower end of @ref SampleRng::RandomColor.
inline constexpr ColorId kFirstBright = kRed;
/// Last non-grey color, the upper end of @ref SampleRng::RandomColor.
inline constexpr ColorId kLastBright = kSky;
}  // namespace sample_color

/// Filled circle in world space.
struct DrawCircle {
  Vec2f center{};                        ///< Center, in meters.
  Scalar radius{};                       ///< Radius, in meters.
  ColorId color = sample_color::kWhite;  ///< Fill color.
};

/// Filled axis-aligned box in world space.
struct DrawBox {
  Vec2f center{};                        ///< Center, in meters.
  Vec2f half_size{};                     ///< Half width and half height.
  ColorId color = sample_color::kWhite;  ///< Fill color.
};

/// Line segment in world space.
struct DrawLine {
  Vec2f a{};                             ///< First end, in meters.
  Vec2f b{};                             ///< Second end, in meters.
  ColorId color = sample_color::kWhite;  ///< Line color.
};

/// What a sample shows for one frame, in world space.
///
/// This is the only thing a sample tells the host about its looks: shapes in
/// meters with y pointing up (like the physics world), and palette colors. A
/// @ref SampleDrawer maps them to the screen through @ref camera (flipping y,
/// since screens are y-down) and draws circles, then boxes, then lines (later
/// shapes on top). Keeping drawing platform-neutral lets the same
/// sample run on PC (SDL renderer) and on the DS (hardware sprites and a
/// bitmap background).
///
/// Shapes are plain values pushed in @ref Sample::Draw, as follows:
/// @code
/// draw_list.circles.push_back({body.position, radius, sample_color::kRed});
/// draw_list.boxes.push_back({center, half_size, sample_color::kGreen});
/// draw_list.lines.push_back({from, to, sample_color::kGrey});
/// @endcode
///
/// @note On the DS, small circles and boxes become hardware sprites (128 at
/// most, extra ones are dropped) and the larger ones and the lines are drawn
/// in a bitmap background. Keep the shape counts within the
/// @ref sample_config capacities.
struct DrawList {
  /// World rectangle to show, set by @ref SampleRegistry::Draw to the
  /// @ref SampleContext::world_bounds.
  Aabbf camera{};
  std::vector<DrawCircle> circles;  ///< @ref DrawCircle list, drawn first.
  std::vector<DrawBox> boxes;       ///< @ref DrawBox list, over the circles.
  std::vector<DrawLine> lines;      ///< @ref DrawLine list, drawn last.

  /// Reserves room for every shape list.
  ///
  /// Called once by the host with the @ref sample_config capacities:
  /// @ref Clear keeps the capacity, so frames don't allocate.
  ///
  /// @param circle_count Capacity of @ref circles.
  /// @param box_count Capacity of @ref boxes.
  /// @param line_count Capacity of @ref lines.
  void Reserve(std::size_t circle_count, std::size_t box_count,
               std::size_t line_count) {
    circles.reserve(circle_count);
    boxes.reserve(box_count);
    lines.reserve(line_count);
  }

  /// Removes every shape, keeping the capacity.
  void Clear() {
    circles.clear();
    boxes.clear();
    lines.clear();
  }

  /// Returns true if the list holds no shape.
  [[nodiscard]] bool empty() const {
    return circles.empty() && boxes.empty() && lines.empty();
  }
};

/// Platform backend that renders a @ref DrawList.
///
/// Implemented by the hosts, not by the samples: `SdlSampleDrawer`
/// (`main/sdl/`) and `NdsSampleDrawer` (`nds/samples/`). A new sample never
/// needs a new drawer.
class SampleDrawer {
 public:
  /// Destroys the drawer.
  virtual ~SampleDrawer() = default;
  /// Draws one frame.
  ///
  /// @param draw_list The shapes to draw, mapped from its camera rectangle to
  /// the screen.
  virtual void Render(const DrawList& draw_list) = 0;
};

}  // namespace neko

#endif  // MAIN_SAMPLES_DRAW_LIST_H_
