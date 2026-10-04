#ifndef NDS_SAMPLES_NDS_SAMPLE_DRAWER_H_
#define NDS_SAMPLES_NDS_SAMPLE_DRAWER_H_

#include "main/samples/draw_list.h"

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
        int sprites_used = 0;
        /**
         * @brief Shapes that needed a sprite past the 128th, in the last Render
         */
        int dropped_sprites = 0;
        int bitmap_shapes = 0;
        /**
         * @brief Number of bitmap redraws since Init
         */
        std::uint32_t bitmap_redraws = 0;
    };

    /**
     * @brief Video modes, VRAM banks A (BG3 bitmap) and B (sprites), palettes, circle graphics. Leaves the sub engine
     * and bank C to the console.
     */
    void Init();
    void Render(const DrawList& draw_list) override;
    /**
     * @brief Call right after swiWaitForVBlank: OAM update and bitmap buffer swap, during VBlank
     */
    void Present();
    [[nodiscard]] const Stats& stats() const { return stats_; }

private:
    static constexpr int kMaxSpriteRadius = 32;
    static constexpr int kMaxSpriteBox = 64;
    static constexpr std::size_t kMaxBoxGfx = 16;

    enum class BitmapShapeType : std::uint8_t
    {
        kRect,
        kCircle,
        kLine
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
        ColorId color = sample_color::kEmpty;
        BitmapShapeType type = BitmapShapeType::kRect;
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
     * @brief Indexed by radius in pixels, from 1 to kMaxSpriteRadius
     */
    std::array<SpriteGfx, kMaxSpriteRadius + 1> circle_gfx_{};
    std::array<BoxGfx, kMaxBoxGfx> box_gfx_{};
    std::size_t box_gfx_count_ = 0;
    int previous_sprite_count_ = 0;

    int bg_ = -1;
    /**
     * @brief Index (0 or 1) of the buffer on screen: buffer i is at map base 4 * i (64 KB each in bank A)
     */
    int front_buffer_ = 0;
    bool swap_pending_ = false;
    std::uint32_t front_hash_ = 0;
    std::vector<BitmapShape> bitmap_shapes_;
};

} // namespace neko

#endif  // NDS_SAMPLES_NDS_SAMPLE_DRAWER_H_
