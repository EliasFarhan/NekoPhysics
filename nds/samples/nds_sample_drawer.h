#pragma once

#include "samples/draw_list.h"

#include <nds.h>

#include <array>
#include <optional>
#include <cstdint>
#include <vector>

namespace neko
{

/**
 * @brief Renders a DrawList on the DS top screen (main engine), with no float:
 * - circles up to 32 px of radius and boxes up to 64x64 px are hardware sprites (at most 128, the rest are dropped);
 * - bigger shapes and lines go to a double-buffered 256x256 8bpp bitmap on BG3, redrawn only when its content
 *   changes (hash of the bitmap-routed shapes).
 * Sprites are above the bitmap. Render builds the next frame during the visible frame, Present (right after
 * swiWaitForVBlank) sends it to the hardware.
 */
class NdsSampleDrawer final : public SampleDrawer
{
public:
    struct Stats
    {
        int spritesUsed = 0;
        /**
         * @brief Shapes that needed a sprite past the 128th, in the last Render
         */
        int droppedSprites = 0;
        int bitmapShapes = 0;
        /**
         * @brief Number of bitmap redraws since Init
         */
        std::uint32_t bitmapRedraws = 0;
    };

    /**
     * @brief Video modes, VRAM banks A (BG3 bitmap) and B (sprites), palettes, circle graphics. Leaves the sub engine
     * and bank C to the console.
     */
    void Init();
    void Render(const DrawList& drawList) override;
    /**
     * @brief Call right after swiWaitForVBlank: OAM update and bitmap buffer swap, during VBlank
     */
    void Present();
    [[nodiscard]] const Stats& stats() const { return stats_; }

private:
    static constexpr int MAX_SPRITE_RADIUS = 32;
    static constexpr int MAX_SPRITE_BOX = 64;
    static constexpr std::size_t MAX_BOX_GFX = 16;

    enum class BitmapShapeType : std::uint8_t
    {
        RECT,
        CIRCLE,
        LINE
    };
    /**
     * @brief In pixels. RECT: [x0, x1) x [y0, y1); CIRCLE: center (x0, y0), radius x1; LINE: (x0, y0) to (x1, y1)
     */
    struct BitmapShape
    {
        int x0 = 0;
        int y0 = 0;
        int x1 = 0;
        int y1 = 0;
        ColorId color = sample_color::EMPTY;
        BitmapShapeType type = BitmapShapeType::RECT;
    };
    struct SpriteShape
    {
        int width = 8;
        int height = 8;
        SpriteSize size = SpriteSize_8x8;
    };
    struct SpriteGfx
    {
        SpriteShape shape{};
        u16* gfx = nullptr;
    };
    struct BoxGfx
    {
        int width = 0;
        int height = 0;
        SpriteGfx sprite{};
    };

    void AddSprite(int x, int y, const SpriteGfx& sprite, ColorId color);
    [[nodiscard]] static std::optional<SpriteShape> FindSpriteShape(int width, int height);
    [[nodiscard]] const BoxGfx* GetBoxGfx(int width, int height);
    [[nodiscard]] std::uint32_t HashBitmapShapes() const;
    void RedrawBitmap(u16* buffer) const;

    Stats stats_{};
    /**
     * @brief Indexed by radius in pixels, from 1 to MAX_SPRITE_RADIUS
     */
    std::array<SpriteGfx, MAX_SPRITE_RADIUS + 1> circleGfx_{};
    std::array<BoxGfx, MAX_BOX_GFX> boxGfx_{};
    std::size_t boxGfxCount_ = 0;
    int previousSpriteCount_ = 0;

    int bg_ = -1;
    /**
     * @brief Index (0 or 1) of the buffer on screen: buffer i is at map base 4 * i (64 KB each in bank A)
     */
    int frontBuffer_ = 0;
    bool swapPending_ = false;
    std::uint32_t frontHash_ = 0;
    std::vector<BitmapShape> bitmapShapes_;
};

} // namespace neko
