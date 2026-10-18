#include "nds/samples/nds_sample_drawer.h"

#include <algorithm>
#include <cstdlib>

#include "neko/math/scalar_traits.h"

namespace neko {

static_assert(ScalarTraits<Scalar>::kIsFixedPoint &&
                  ScalarTraits<Scalar>::kDisplayName == "Fixed16",
              "The DS drawer maps Fixed16 raw values to pixels, with no float");

namespace {

constexpr int kSpriteCountMax = 128;
// Pixel coordinates are clamped to this range, so that a body far away can't
// make a line loop for long
constexpr std::int64_t kPixelLimit = 2048;
constexpr int kBufferMapBaseStride = 4;
// 64 KB per 256x256 8bpp buffer, in u16
constexpr std::size_t kBufferHalfwords = 256 * 256 / 2;
constexpr std::uint32_t kVisibleBitmapBytes = 256 * 192;

constexpr u16 ToRgb15(const PaletteColor& color) {
  return static_cast<u16>((color.r >> 3) | ((color.g >> 3) << 5) |
                          ((color.b >> 3) << 10));
}

// Letterbox background: bitmap palette entry 0 is transparent and shows the
// backdrop color
constexpr u16 kBackdropColor = ToRgb15({20, 20, 26});

// Round(value * scale) with value and scale in 16.16, computed in 64 bits so
// that it can't overflow
int ToPixels(Scalar value, Scalar scale) {
  const std::int64_t product = std::int64_t{value.underlying_value()} *
                               std::int64_t{scale.underlying_value()};
  const std::int64_t pixels = (product + (std::int64_t{1} << 31)) >> 32;
  return static_cast<int>(std::clamp(pixels, -kPixelLimit, kPixelLimit));
}

// Writes 4bpp sprite graphics (1D mapping: 8x8 tiles in row-major order, low
// nibble = left pixel) with 16-bit VRAM writes. Pixels where inside(x, y) is
// true use palette entry 1, the others are transparent.
template <typename Inside>
void RasterizeSprite(u16* gfx, int width, int height, Inside inside) {
  volatile u16* out = gfx;
  const int tiles_x = width / 8;
  for (int tile_y = 0; tile_y < height / 8; ++tile_y) {
    for (int tile_x = 0; tile_x < tiles_x; ++tile_x) {
      for (int row = 0; row < 8; ++row) {
        for (int half = 0; half < 2; ++half) {
          unsigned value = 0;
          for (int i = 0; i < 4; ++i) {
            if (inside(tile_x * 8 + half * 4 + i, tile_y * 8 + row)) {
              value |= 1u << (4 * i);
            }
          }
          *out++ = static_cast<u16>(value);
        }
      }
    }
  }
}

// Pixel of a circle of radius r (in pixels) whose bounding square starts at 0:
// pixel centers within r of the center (r, r)
constexpr bool InsideCircle(int x, int y, int radius) {
  const int dx = 2 * x + 1 - 2 * radius;
  const int dy = 2 * y + 1 - 2 * radius;
  return dx * dx + dy * dy <= 4 * radius * radius;
}

// VRAM has no 8-bit writes: every 8bpp pixel write is a 16-bit
// read-modify-write of its pair, through volatile so the compiler can't narrow
// it to a byte store
void PutPixel(volatile u16* buffer, int x, int y, ColorId color) {
  volatile u16& pair = buffer[(y * 256 + x) >> 1];
  const unsigned current = pair;
  pair = static_cast<u16>((x & 1) != 0
                              ? (current & 0x00FFu) | (unsigned{color} << 8)
                              : (current & 0xFF00u) | unsigned{color});
}

// Row y, pixels [x0, x1), already clipped
void FillSpan(volatile u16* buffer, int x0, int x1, int y, ColorId color) {
  if (x0 >= x1) {
    return;
  }
  if ((x0 & 1) != 0) {
    PutPixel(buffer, x0, y, color);
    ++x0;
  }
  const auto both = static_cast<u16>(unsigned{color} | (unsigned{color} << 8));
  for (; x0 + 1 < x1; x0 += 2) {
    buffer[(y * 256 + x0) >> 1] = both;
  }
  if (x0 < x1) {
    PutPixel(buffer, x0, y, color);
  }
}

std::uint32_t HashInt(std::uint32_t hash, int value) {
  // FNV-1a over the 4 bytes
  auto bits = static_cast<std::uint32_t>(value);
  for (int i = 0; i < 4; ++i) {
    hash ^= bits & 0xFFu;
    hash *= 16777619u;
    bits >>= 8;
  }
  return hash;
}

}  // namespace

std::optional<NdsSampleDrawer::SpriteShape> NdsSampleDrawer::FindSpriteShape(
    int width, int height) {
  // Every sprite shape, smallest area first
  constexpr std::array<SpriteShape, 12> shapes = {{
      {8, 8, SpriteSize_8x8},
      {16, 8, SpriteSize_16x8},
      {8, 16, SpriteSize_8x16},
      {16, 16, SpriteSize_16x16},
      {32, 8, SpriteSize_32x8},
      {8, 32, SpriteSize_8x32},
      {32, 16, SpriteSize_32x16},
      {16, 32, SpriteSize_16x32},
      {32, 32, SpriteSize_32x32},
      {64, 32, SpriteSize_64x32},
      {32, 64, SpriteSize_32x64},
      {64, 64, SpriteSize_64x64},
  }};
  for (const auto& shape : shapes) {
    if (shape.width >= width && shape.height >= height) {
      return shape;
    }
  }
  return std::nullopt;
}

void NdsSampleDrawer::Init() {
  videoSetMode(MODE_5_2D);
  vramSetBankA(VRAM_A_MAIN_BG);
  vramSetBankB(VRAM_B_MAIN_SPRITE);
  // 1D_64 rather than 1D_32: the 10-bit tile index only addresses 32 KB of
  // sprite VRAM with 32-byte tiles, and the circle graphics need ~37.5 KB
  oamInit(&oamMain, SpriteMapping_1D_64, false);

  bg_ = bgInit(3, BgType_Bmp8, BgSize_B8_256x256, 0, 0);
  bgSetPriority(bg_, 3);
  dmaFillWords(0, BG_GFX, 2 * kBufferHalfwords * sizeof(u16));
  bgSetMapBase(bg_, 0);
  front_buffer_ = 0;
  swap_pending_ = false;

  BG_PALETTE[0] = kBackdropColor;
  for (std::size_t color = 1; color < kSamplePalette.size(); ++color) {
    const u16 rgb = ToRgb15(kSamplePalette[color]);
    BG_PALETTE[color] = rgb;
    SPRITE_PALETTE[color * 16 + 1] = rgb;
  }

  for (int radius = 1; radius <= kMaxSpriteRadius; ++radius) {
    // 2 * radius <= 64: always found
    const auto shape =
        FindSpriteShape(2 * radius, 2 * radius).value_or(SpriteShape{});
    u16* gfx = oamAllocateGfx(&oamMain, shape.size, SpriteColorFormat_16Color);
    RasterizeSprite(gfx, shape.width, shape.height, [radius](int x, int y) {
      return InsideCircle(x, y, radius);
    });
    circle_gfx_[static_cast<std::size_t>(radius)] = {shape, gfx};
  }

  bitmap_shapes_.reserve(512);
  front_hash_ = HashBitmapShapes();
  stats_ = {};
  previous_sprite_count_ = 0;
}

void NdsSampleDrawer::AddSprite(int x, int y, const SpriteGfx& sprite,
                                ColorId color) {
  if (x >= SCREEN_WIDTH || x + sprite.shape.width <= 0 || y >= SCREEN_HEIGHT ||
      y + sprite.shape.height <= 0) {
    return;
  }
  if (stats_.sprites_used >= kSpriteCountMax) {
    ++stats_.dropped_sprites;
    return;
  }
  // x and y are 9 and 8 bits in OAM: negative values wrap, which shows the part
  // of the sprite that is on screen
  oamSet(&oamMain, stats_.sprites_used, x, y, 0, color, sprite.shape.size,
         SpriteColorFormat_16Color, sprite.gfx, -1, false, false, false, false,
         false);
  ++stats_.sprites_used;
}

const NdsSampleDrawer::BoxGfx* NdsSampleDrawer::GetBoxGfx(int width,
                                                          int height) {
  for (std::size_t i = 0; i < box_gfx_count_; ++i) {
    if (box_gfx_[i].width == width && box_gfx_[i].height == height) {
      return &box_gfx_[i];
    }
  }
  const auto shape = FindSpriteShape(width, height);
  if (!shape.has_value() || box_gfx_count_ == box_gfx_.size()) {
    return nullptr;
  }
  u16* gfx = oamAllocateGfx(&oamMain, shape->size, SpriteColorFormat_16Color);
  if (gfx == nullptr) {
    return nullptr;
  }
  RasterizeSprite(
      gfx, shape->width, shape->height,
      [width, height](int x, int y) { return x < width && y < height; });
  box_gfx_[box_gfx_count_] = {width, height, {*shape, gfx}};
  return &box_gfx_[box_gfx_count_++];
}

void NdsSampleDrawer::Render(const DrawList& draw_list) {
  // The world is y-up and the screen y-down: the camera's top-left corner,
  // (min x, max y), is pixel (0, 0)
  const Scalar left = draw_list.camera.min_bound().x();
  const Scalar top = draw_list.camera.max_bound().y();
  const Scalar camera_width = draw_list.camera.max_bound().x() - left;
  const Scalar scale = Scalar{SCREEN_WIDTH} / camera_width;
  const auto to_screen_x = [left, scale](Scalar x) {
    return ToPixels(x - left, scale);
  };
  const auto to_screen_y = [top, scale](Scalar y) {
    return ToPixels(top - y, scale);
  };

  stats_.sprites_used = 0;
  stats_.dropped_sprites = 0;
  bitmap_shapes_.clear();

  for (const auto& circle : draw_list.circles) {
    const int radius = std::max(1, ToPixels(circle.radius, scale));
    const int x = to_screen_x(circle.center.x());
    const int y = to_screen_y(circle.center.y());
    if (radius <= kMaxSpriteRadius) {
      AddSprite(x - radius, y - radius,
                circle_gfx_[static_cast<std::size_t>(radius)], circle.color);
    } else {
      bitmap_shapes_.push_back(
          {x, y, radius, 0, circle.color, BitmapShapeType::kCircle});
    }
  }
  for (const auto& box : draw_list.boxes) {
    const int x0 = to_screen_x(box.center.x() - box.half_size.x());
    const int y0 = to_screen_y(box.center.y() + box.half_size.y());
    const int width =
        std::max(1, to_screen_x(box.center.x() + box.half_size.x()) - x0);
    const int height =
        std::max(1, to_screen_y(box.center.y() - box.half_size.y()) - y0);
    const BoxGfx* gfx = width <= kMaxSpriteBox && height <= kMaxSpriteBox
                            ? GetBoxGfx(width, height)
                            : nullptr;
    if (gfx != nullptr) {
      AddSprite(x0, y0, gfx->sprite, box.color);
    } else {
      bitmap_shapes_.push_back(
          {x0, y0, x0 + width, y0 + height, box.color, BitmapShapeType::kRect});
    }
  }
  for (const auto& line : draw_list.lines) {
    bitmap_shapes_.push_back({to_screen_x(line.a.x()), to_screen_y(line.a.y()),
                              to_screen_x(line.b.x()), to_screen_y(line.b.y()),
                              line.color, BitmapShapeType::kLine});
  }

  if (stats_.sprites_used < previous_sprite_count_) {
    oamClear(&oamMain, stats_.sprites_used,
             previous_sprite_count_ - stats_.sprites_used);
  }
  previous_sprite_count_ = stats_.sprites_used;

  stats_.bitmap_shapes = static_cast<int>(bitmap_shapes_.size());
  const std::uint32_t hash = HashBitmapShapes();
  if (hash != front_hash_) {
    RedrawBitmap(BG_GFX + kBufferHalfwords *
                              static_cast<std::size_t>(1 - front_buffer_));
    front_hash_ = hash;
    swap_pending_ = true;
    ++stats_.bitmap_redraws;
  }
}

void NdsSampleDrawer::Present() {
  oamUpdate(&oamMain);
  if (swap_pending_) {
    front_buffer_ = 1 - front_buffer_;
    bgSetMapBase(bg_,
                 static_cast<unsigned>(kBufferMapBaseStride * front_buffer_));
    swap_pending_ = false;
  }
}

std::uint32_t NdsSampleDrawer::HashBitmapShapes() const {
  std::uint32_t hash = 2166136261u;
  hash = HashInt(hash, static_cast<int>(bitmap_shapes_.size()));
  for (const auto& shape : bitmap_shapes_) {
    hash = HashInt(hash, shape.x0);
    hash = HashInt(hash, shape.y0);
    hash = HashInt(hash, shape.x1);
    hash = HashInt(hash, shape.y1);
    hash = HashInt(hash, (static_cast<int>(shape.type) << 8) | shape.color);
  }
  return hash;
}

void NdsSampleDrawer::RedrawBitmap(u16* buffer) const {
  dmaFillWords(0, buffer, kVisibleBitmapBytes);
  volatile u16* pixels = buffer;
  for (const auto& shape : bitmap_shapes_) {
    switch (shape.type) {
      case BitmapShapeType::kRect: {
        const int x0 = std::max(shape.x0, 0);
        const int x1 = std::min(shape.x1, SCREEN_WIDTH);
        for (int y = std::max(shape.y0, 0);
             y < std::min(shape.y1, SCREEN_HEIGHT); ++y) {
          FillSpan(pixels, x0, x1, y, shape.color);
        }
        break;
      }
      case BitmapShapeType::kCircle: {
        const int radius = shape.x1;
        const int left = shape.x0 - radius;
        const int top = shape.y0 - radius;
        for (int row = 0; row < 2 * radius; ++row) {
          const int y = top + row;
          if (y < 0 || y >= SCREEN_HEIGHT) {
            continue;
          }
          int column = 0;
          while (column < radius && !InsideCircle(column, row, radius)) {
            ++column;
          }
          FillSpan(pixels, std::max(left + column, 0),
                   std::min(left + 2 * radius - column, SCREEN_WIDTH), y,
                   shape.color);
        }
        break;
      }
      case BitmapShapeType::kLine: {
        if (shape.y0 == shape.y1) {
          // Horizontal: two pixels per 16-bit write
          if (shape.y0 >= 0 && shape.y0 < SCREEN_HEIGHT) {
            FillSpan(pixels, std::max(std::min(shape.x0, shape.x1), 0),
                     std::min(std::max(shape.x0, shape.x1) + 1, SCREEN_WIDTH),
                     shape.y0, shape.color);
          }
          break;
        }
        if (shape.x0 == shape.x1) {
          if (shape.x0 >= 0 && shape.x0 < SCREEN_WIDTH) {
            const int y1 =
                std::min(std::max(shape.y0, shape.y1), SCREEN_HEIGHT - 1);
            for (int y = std::max(std::min(shape.y0, shape.y1), 0); y <= y1;
                 ++y) {
              PutPixel(pixels, shape.x0, y, shape.color);
            }
          }
          break;
        }
        // Bresenham, clipped per pixel
        int x = shape.x0;
        int y = shape.y0;
        const int dx = std::abs(shape.x1 - shape.x0);
        const int dy = -std::abs(shape.y1 - shape.y0);
        const int step_x = shape.x0 < shape.x1 ? 1 : -1;
        const int step_y = shape.y0 < shape.y1 ? 1 : -1;
        int error = dx + dy;
        while (true) {
          if (x >= 0 && x < SCREEN_WIDTH && y >= 0 && y < SCREEN_HEIGHT) {
            PutPixel(pixels, x, y, shape.color);
          }
          if (x == shape.x1 && y == shape.y1) {
            break;
          }
          const int double_error = 2 * error;
          if (double_error >= dy) {
            error += dy;
            x += step_x;
          }
          if (double_error <= dx) {
            error += dx;
            y += step_y;
          }
        }
        break;
      }
    }
  }
}

}  // namespace neko
