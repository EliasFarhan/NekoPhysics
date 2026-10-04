#pragma once

#include "neko/math/shape.h"
#include "neko/math/vec2.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace neko
{

/**
 * @brief Index into kSamplePalette. 0 is transparent (the DS sprite and bitmap palettes reserve entry 0).
 */
using ColorId = std::uint8_t;

struct PaletteColor
{
    std::uint8_t r = 0;
    std::uint8_t g = 0;
    std::uint8_t b = 0;
};

/**
 * @brief Colors shared by every backend: on the DS, ColorId i is sprite palette bank i (fill = entry 1) and bitmap
 * BG palette entry i, so no backend has to quantize anything
 */
inline constexpr std::array<PaletteColor, 16> kSamplePalette = { {
    { 0, 0, 0 },       // transparent
    { 255, 255, 255 }, // white
    { 160, 160, 160 }, // grey
    { 80, 80, 80 },    // dark grey
    { 230, 60, 60 },   // red
    { 60, 200, 80 },   // green
    { 60, 110, 230 },  // blue
    { 240, 220, 60 },  // yellow
    { 60, 210, 220 },  // cyan
    { 220, 70, 200 },  // magenta
    { 245, 150, 50 },  // orange
    { 150, 90, 230 },  // purple
    { 245, 150, 190 }, // pink
    { 170, 230, 70 },  // lime
    { 40, 160, 140 },  // teal
    { 130, 190, 250 }, // sky
} };

namespace sample_color
{
// Not TRANSPARENT: wingdi.h defines it as a macro
inline constexpr ColorId EMPTY = 0;
inline constexpr ColorId WHITE = 1;
inline constexpr ColorId GREY = 2;
inline constexpr ColorId DARK_GREY = 3;
inline constexpr ColorId RED = 4;
inline constexpr ColorId GREEN = 5;
inline constexpr ColorId BLUE = 6;
inline constexpr ColorId YELLOW = 7;
inline constexpr ColorId CYAN = 8;
inline constexpr ColorId MAGENTA = 9;
inline constexpr ColorId ORANGE = 10;
inline constexpr ColorId PURPLE = 11;
inline constexpr ColorId PINK = 12;
inline constexpr ColorId LIME = 13;
inline constexpr ColorId TEAL = 14;
inline constexpr ColorId SKY = 15;
/**
 * @brief Non-grey entries, the range random colors are picked from
 */
inline constexpr ColorId FIRST_BRIGHT = RED;
inline constexpr ColorId LAST_BRIGHT = SKY;
} // namespace sample_color

struct DrawCircle
{
    Vec2f center{};
    Scalar radius{};
    ColorId color = sample_color::WHITE;
};

struct DrawBox
{
    Vec2f center{};
    Vec2f halfSize{};
    ColorId color = sample_color::WHITE;
};

struct DrawLine
{
    Vec2f a{};
    Vec2f b{};
    ColorId color = sample_color::WHITE;
};

/**
 * @brief What a sample shows for one frame, in world space (meters, y down). Backends map it to the screen and
 * draw circles, then boxes, then lines.
 */
struct DrawList
{
    /**
     * @brief World rectangle to show: the SampleContext bounds
     */
    Aabbf camera{};
    std::vector<DrawCircle> circles;
    std::vector<DrawBox> boxes;
    std::vector<DrawLine> lines;

    /**
     * @brief Called once by the host: Clear keeps the capacity, so frames don't allocate
     */
    void Reserve(std::size_t circleCount, std::size_t boxCount, std::size_t lineCount)
    {
        circles.reserve(circleCount);
        boxes.reserve(boxCount);
        lines.reserve(lineCount);
    }

    void Clear()
    {
        circles.clear();
        boxes.clear();
        lines.clear();
    }

    [[nodiscard]] bool empty() const { return circles.empty() && boxes.empty() && lines.empty(); }
};

class SampleDrawer
{
public:
    virtual ~SampleDrawer() = default;
    virtual void Render(const DrawList& drawList) = 0;
};

} // namespace neko
