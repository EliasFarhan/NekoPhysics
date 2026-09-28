#include "nds_sample_drawer.h"

#include <algorithm>
#include <cstdlib>
#include <type_traits>

namespace neko
{

static_assert(std::is_same_v<Scalar, Fixed16>, "The DS drawer maps Fixed16 raw values to pixels, with no float");

namespace
{

constexpr int SPRITE_COUNT_MAX = 128;
/**
 * @brief Pixel coordinates are clamped to this range, so that a body far away can't make a line loop for long
 */
constexpr std::int64_t PIXEL_LIMIT = 2048;
constexpr int BUFFER_MAP_BASE_STRIDE = 4;
/**
 * @brief 64 KB per 256x256 8bpp buffer, in u16
 */
constexpr std::size_t BUFFER_HALFWORDS = 256 * 256 / 2;
constexpr std::uint32_t VISIBLE_BITMAP_BYTES = 256 * 192;

constexpr u16 ToRgb15(const PaletteColor& color)
{
    return static_cast<u16>((color.r >> 3) | ((color.g >> 3) << 5) | ((color.b >> 3) << 10));
}

/**
 * @brief Letterbox background: bitmap palette entry 0 is transparent and shows the backdrop color
 */
constexpr u16 BACKDROP_COLOR = ToRgb15({ 20, 20, 26 });

/**
 * @brief Round(value * scale) with value and scale in 16.16, computed in 64 bits so that it can't overflow
 */
int ToPixels(Scalar value, Scalar scale)
{
    const std::int64_t product = std::int64_t{ value.underlyingValue() } * std::int64_t{ scale.underlyingValue() };
    const std::int64_t pixels = (product + (std::int64_t{ 1 } << 31)) >> 32;
    return static_cast<int>(std::clamp(pixels, -PIXEL_LIMIT, PIXEL_LIMIT));
}

/**
 * @brief Writes 4bpp sprite graphics (1D mapping: 8x8 tiles in row-major order, low nibble = left pixel) with
 * 16-bit VRAM writes. Pixels where inside(x, y) is true use palette entry 1, the others are transparent.
 */
template<typename Inside>
void RasterizeSprite(u16* gfx, int width, int height, Inside inside)
{
    volatile u16* out = gfx;
    const int tilesX = width / 8;
    for (int tileY = 0; tileY < height / 8; ++tileY)
    {
        for (int tileX = 0; tileX < tilesX; ++tileX)
        {
            for (int row = 0; row < 8; ++row)
            {
                for (int half = 0; half < 2; ++half)
                {
                    unsigned value = 0;
                    for (int i = 0; i < 4; ++i)
                    {
                        if (inside(tileX * 8 + half * 4 + i, tileY * 8 + row))
                        {
                            value |= 1u << (4 * i);
                        }
                    }
                    *out++ = static_cast<u16>(value);
                }
            }
        }
    }
}

/**
 * @brief Pixel of a circle of radius r (in pixels) whose bounding square starts at 0: pixel centers within r of the
 * center (r, r)
 */
constexpr bool InsideCircle(int x, int y, int radius)
{
    const int dx = 2 * x + 1 - 2 * radius;
    const int dy = 2 * y + 1 - 2 * radius;
    return dx * dx + dy * dy <= 4 * radius * radius;
}

// VRAM has no 8-bit writes: every 8bpp pixel write is a 16-bit read-modify-write of its pair, through volatile so
// the compiler can't narrow it to a byte store
void PutPixel(volatile u16* buffer, int x, int y, ColorId color)
{
    volatile u16& pair = buffer[(y * 256 + x) >> 1];
    const unsigned current = pair;
    pair = static_cast<u16>((x & 1) != 0 ? (current & 0x00FFu) | (unsigned{ color } << 8)
                                         : (current & 0xFF00u) | unsigned{ color });
}

/**
 * @brief Row y, pixels [x0, x1), already clipped
 */
void FillSpan(volatile u16* buffer, int x0, int x1, int y, ColorId color)
{
    if (x0 >= x1)
    {
        return;
    }
    if ((x0 & 1) != 0)
    {
        PutPixel(buffer, x0, y, color);
        ++x0;
    }
    const auto both = static_cast<u16>(unsigned{ color } | (unsigned{ color } << 8));
    for (; x0 + 1 < x1; x0 += 2)
    {
        buffer[(y * 256 + x0) >> 1] = both;
    }
    if (x0 < x1)
    {
        PutPixel(buffer, x0, y, color);
    }
}

std::uint32_t HashInt(std::uint32_t hash, int value)
{
    // FNV-1a over the 4 bytes
    auto bits = static_cast<std::uint32_t>(value);
    for (int i = 0; i < 4; ++i)
    {
        hash ^= bits & 0xFFu;
        hash *= 16777619u;
        bits >>= 8;
    }
    return hash;
}

} // namespace

std::optional<NdsSampleDrawer::SpriteShape> NdsSampleDrawer::FindSpriteShape(int width, int height)
{
    // Every sprite shape, smallest area first
    constexpr std::array<SpriteShape, 12> shapes = { {
        { 8, 8, SpriteSize_8x8 },
        { 16, 8, SpriteSize_16x8 },
        { 8, 16, SpriteSize_8x16 },
        { 16, 16, SpriteSize_16x16 },
        { 32, 8, SpriteSize_32x8 },
        { 8, 32, SpriteSize_8x32 },
        { 32, 16, SpriteSize_32x16 },
        { 16, 32, SpriteSize_16x32 },
        { 32, 32, SpriteSize_32x32 },
        { 64, 32, SpriteSize_64x32 },
        { 32, 64, SpriteSize_32x64 },
        { 64, 64, SpriteSize_64x64 },
    } };
    for (const auto& shape : shapes)
    {
        if (shape.width >= width && shape.height >= height)
        {
            return shape;
        }
    }
    return std::nullopt;
}

void NdsSampleDrawer::Init()
{
    videoSetMode(MODE_5_2D);
    vramSetBankA(VRAM_A_MAIN_BG);
    vramSetBankB(VRAM_B_MAIN_SPRITE);
    // 1D_64 rather than 1D_32: the 10-bit tile index only addresses 32 KB of sprite VRAM with 32-byte tiles, and the
    // circle graphics need ~37.5 KB
    oamInit(&oamMain, SpriteMapping_1D_64, false);

    bg_ = bgInit(3, BgType_Bmp8, BgSize_B8_256x256, 0, 0);
    bgSetPriority(bg_, 3);
    dmaFillWords(0, BG_GFX, 2 * BUFFER_HALFWORDS * sizeof(u16));
    bgSetMapBase(bg_, 0);
    frontBuffer_ = 0;
    swapPending_ = false;

    BG_PALETTE[0] = BACKDROP_COLOR;
    for (std::size_t color = 1; color < kSamplePalette.size(); ++color)
    {
        const u16 rgb = ToRgb15(kSamplePalette[color]);
        BG_PALETTE[color] = rgb;
        SPRITE_PALETTE[color * 16 + 1] = rgb;
    }

    for (int radius = 1; radius <= MAX_SPRITE_RADIUS; ++radius)
    {
        // 2 * radius <= 64: always found
        const auto shape = FindSpriteShape(2 * radius, 2 * radius).value_or(SpriteShape{});
        u16* gfx = oamAllocateGfx(&oamMain, shape.size, SpriteColorFormat_16Color);
        RasterizeSprite(gfx, shape.width, shape.height,
            [radius](int x, int y) { return InsideCircle(x, y, radius); });
        circleGfx_[static_cast<std::size_t>(radius)] = { shape, gfx };
    }

    bitmapShapes_.reserve(512);
    frontHash_ = HashBitmapShapes();
    stats_ = {};
    previousSpriteCount_ = 0;
}

void NdsSampleDrawer::AddSprite(int x, int y, const SpriteGfx& sprite, ColorId color)
{
    if (x >= SCREEN_WIDTH || x + sprite.shape.width <= 0 || y >= SCREEN_HEIGHT || y + sprite.shape.height <= 0)
    {
        return;
    }
    if (stats_.spritesUsed >= SPRITE_COUNT_MAX)
    {
        ++stats_.droppedSprites;
        return;
    }
    // x and y are 9 and 8 bits in OAM: negative values wrap, which shows the part of the sprite that is on screen
    oamSet(&oamMain, stats_.spritesUsed, x, y, 0, color, sprite.shape.size, SpriteColorFormat_16Color, sprite.gfx, -1,
        false, false, false, false, false);
    ++stats_.spritesUsed;
}

const NdsSampleDrawer::BoxGfx* NdsSampleDrawer::GetBoxGfx(int width, int height)
{
    for (std::size_t i = 0; i < boxGfxCount_; ++i)
    {
        if (boxGfx_[i].width == width && boxGfx_[i].height == height)
        {
            return &boxGfx_[i];
        }
    }
    const auto shape = FindSpriteShape(width, height);
    if (!shape.has_value() || boxGfxCount_ == boxGfx_.size())
    {
        return nullptr;
    }
    u16* gfx = oamAllocateGfx(&oamMain, shape->size, SpriteColorFormat_16Color);
    if (gfx == nullptr)
    {
        return nullptr;
    }
    RasterizeSprite(gfx, shape->width, shape->height,
        [width, height](int x, int y) { return x < width && y < height; });
    boxGfx_[boxGfxCount_] = { width, height, { *shape, gfx } };
    return &boxGfx_[boxGfxCount_++];
}

void NdsSampleDrawer::Render(const DrawList& drawList)
{
    const Vec2f origin = drawList.camera.minBound;
    const Scalar cameraWidth = drawList.camera.maxBound.x - drawList.camera.minBound.x;
    const Scalar scale = Scalar{ SCREEN_WIDTH } / cameraWidth;
    const auto toScreenX = [origin, scale](Scalar x) { return ToPixels(x - origin.x, scale); };
    const auto toScreenY = [origin, scale](Scalar y) { return ToPixels(y - origin.y, scale); };

    stats_.spritesUsed = 0;
    stats_.droppedSprites = 0;
    bitmapShapes_.clear();

    for (const auto& circle : drawList.circles)
    {
        const int radius = std::max(1, ToPixels(circle.radius, scale));
        const int x = toScreenX(circle.center.x);
        const int y = toScreenY(circle.center.y);
        if (radius <= MAX_SPRITE_RADIUS)
        {
            AddSprite(x - radius, y - radius, circleGfx_[static_cast<std::size_t>(radius)], circle.color);
        }
        else
        {
            bitmapShapes_.push_back({ x, y, radius, 0, circle.color, BitmapShapeType::CIRCLE });
        }
    }
    for (const auto& box : drawList.boxes)
    {
        const int x0 = toScreenX(box.center.x - box.halfSize.x);
        const int y0 = toScreenY(box.center.y - box.halfSize.y);
        const int width = std::max(1, toScreenX(box.center.x + box.halfSize.x) - x0);
        const int height = std::max(1, toScreenY(box.center.y + box.halfSize.y) - y0);
        const BoxGfx* gfx = width <= MAX_SPRITE_BOX && height <= MAX_SPRITE_BOX ? GetBoxGfx(width, height) : nullptr;
        if (gfx != nullptr)
        {
            AddSprite(x0, y0, gfx->sprite, box.color);
        }
        else
        {
            bitmapShapes_.push_back({ x0, y0, x0 + width, y0 + height, box.color, BitmapShapeType::RECT });
        }
    }
    for (const auto& line : drawList.lines)
    {
        bitmapShapes_.push_back({ toScreenX(line.a.x), toScreenY(line.a.y), toScreenX(line.b.x), toScreenY(line.b.y),
            line.color, BitmapShapeType::LINE });
    }

    if (stats_.spritesUsed < previousSpriteCount_)
    {
        oamClear(&oamMain, stats_.spritesUsed, previousSpriteCount_ - stats_.spritesUsed);
    }
    previousSpriteCount_ = stats_.spritesUsed;

    stats_.bitmapShapes = static_cast<int>(bitmapShapes_.size());
    const std::uint32_t hash = HashBitmapShapes();
    if (hash != frontHash_)
    {
        RedrawBitmap(BG_GFX + BUFFER_HALFWORDS * static_cast<std::size_t>(1 - frontBuffer_));
        frontHash_ = hash;
        swapPending_ = true;
        ++stats_.bitmapRedraws;
    }
}

void NdsSampleDrawer::Present()
{
    oamUpdate(&oamMain);
    if (swapPending_)
    {
        frontBuffer_ = 1 - frontBuffer_;
        bgSetMapBase(bg_, static_cast<unsigned>(BUFFER_MAP_BASE_STRIDE * frontBuffer_));
        swapPending_ = false;
    }
}

std::uint32_t NdsSampleDrawer::HashBitmapShapes() const
{
    std::uint32_t hash = 2166136261u;
    hash = HashInt(hash, static_cast<int>(bitmapShapes_.size()));
    for (const auto& shape : bitmapShapes_)
    {
        hash = HashInt(hash, shape.x0);
        hash = HashInt(hash, shape.y0);
        hash = HashInt(hash, shape.x1);
        hash = HashInt(hash, shape.y1);
        hash = HashInt(hash, (static_cast<int>(shape.type) << 8) | shape.color);
    }
    return hash;
}

void NdsSampleDrawer::RedrawBitmap(u16* buffer) const
{
    dmaFillWords(0, buffer, VISIBLE_BITMAP_BYTES);
    volatile u16* pixels = buffer;
    for (const auto& shape : bitmapShapes_)
    {
        switch (shape.type)
        {
        case BitmapShapeType::RECT:
        {
            const int x0 = std::max(shape.x0, 0);
            const int x1 = std::min(shape.x1, SCREEN_WIDTH);
            for (int y = std::max(shape.y0, 0); y < std::min(shape.y1, SCREEN_HEIGHT); ++y)
            {
                FillSpan(pixels, x0, x1, y, shape.color);
            }
            break;
        }
        case BitmapShapeType::CIRCLE:
        {
            const int radius = shape.x1;
            const int left = shape.x0 - radius;
            const int top = shape.y0 - radius;
            for (int row = 0; row < 2 * radius; ++row)
            {
                const int y = top + row;
                if (y < 0 || y >= SCREEN_HEIGHT)
                {
                    continue;
                }
                int column = 0;
                while (column < radius && !InsideCircle(column, row, radius))
                {
                    ++column;
                }
                FillSpan(pixels, std::max(left + column, 0), std::min(left + 2 * radius - column, SCREEN_WIDTH), y,
                    shape.color);
            }
            break;
        }
        case BitmapShapeType::LINE:
        {
            // Bresenham, clipped per pixel
            int x = shape.x0;
            int y = shape.y0;
            const int dx = std::abs(shape.x1 - shape.x0);
            const int dy = -std::abs(shape.y1 - shape.y0);
            const int stepX = shape.x0 < shape.x1 ? 1 : -1;
            const int stepY = shape.y0 < shape.y1 ? 1 : -1;
            int error = dx + dy;
            while (true)
            {
                if (x >= 0 && x < SCREEN_WIDTH && y >= 0 && y < SCREEN_HEIGHT)
                {
                    PutPixel(pixels, x, y, shape.color);
                }
                if (x == shape.x1 && y == shape.y1)
                {
                    break;
                }
                const int doubleError = 2 * error;
                if (doubleError >= dy)
                {
                    error += dy;
                    x += stepX;
                }
                if (doubleError <= dx)
                {
                    error += dx;
                    y += stepY;
                }
            }
            break;
        }
        }
    }
}

} // namespace neko
