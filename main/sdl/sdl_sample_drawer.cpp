#include "sdl_sample_drawer.h"

#include "core/profile.h"

#include <SDL3/SDL_log.h>

#include <algorithm>
#include <cmath>
#include <numbers>

namespace neko
{

namespace
{

constexpr int MIN_CIRCLE_SEGMENTS = 8;
constexpr int MAX_CIRCLE_SEGMENTS = 32;
/**
 * @brief Inside of the camera rectangle, so that the letterbox bars show
 */
constexpr SDL_FColor BACKGROUND_COLOR{ 0.08f, 0.08f, 0.1f, 1.0f };

SDL_FColor ToSdlColor(ColorId color)
{
    const auto& c = kSamplePalette[color];
    return { static_cast<float>(c.r) / 255.0f, static_cast<float>(c.g) / 255.0f, static_cast<float>(c.b) / 255.0f,
        1.0f };
}

} // namespace

SdlSampleDrawer::Viewport SdlSampleDrawer::FitCamera(const Aabbf& camera) const
{
    int width = 0;
    int height = 0;
    SDL_GetCurrentRenderOutputSize(renderer_, &width, &height);
    const auto cameraSize = camera.maxBound - camera.minBound;
    const float cameraWidth = float{ cameraSize.x };
    const float cameraHeight = float{ cameraSize.y };
    if (cameraWidth <= 0.0f || cameraHeight <= 0.0f)
    {
        return {};
    }
    const float scale = std::min(static_cast<float>(width) / cameraWidth, static_cast<float>(height) / cameraHeight);
    return { scale, (static_cast<float>(width) - cameraWidth * scale) / 2.0f - float{ camera.minBound.x } * scale,
        (static_cast<float>(height) - cameraHeight * scale) / 2.0f - float{ camera.minBound.y } * scale };
}

void SdlSampleDrawer::Render(const DrawList& drawList)
{
    NEKO_PROFILE_FUNCTION();
    const auto viewport = FitCamera(drawList.camera);

    const SDL_FRect background{ float{ drawList.camera.minBound.x } * viewport.scale + viewport.offsetX,
        float{ drawList.camera.minBound.y } * viewport.scale + viewport.offsetY,
        float{ drawList.camera.maxBound.x - drawList.camera.minBound.x } * viewport.scale,
        float{ drawList.camera.maxBound.y - drawList.camera.minBound.y } * viewport.scale };
    SDL_SetRenderDrawColorFloat(renderer_, BACKGROUND_COLOR.r, BACKGROUND_COLOR.g, BACKGROUND_COLOR.b,
        BACKGROUND_COLOR.a);
    SDL_RenderFillRect(renderer_, &background);

    RenderCircles(drawList, viewport);
    RenderBoxes(drawList, viewport);
    RenderLines(drawList, viewport);
}

void SdlSampleDrawer::RenderCircles(const DrawList& drawList, const Viewport& viewport)
{
    vertices_.clear();
    indices_.clear();
    for (const auto& circle : drawList.circles)
    {
        const SDL_FPoint center{ float{ circle.center.x } * viewport.scale + viewport.offsetX,
            float{ circle.center.y } * viewport.scale + viewport.offsetY };
        const float radius = float{ circle.radius } * viewport.scale;
        const int segments = std::clamp(static_cast<int>(radius), MIN_CIRCLE_SEGMENTS, MAX_CIRCLE_SEGMENTS);
        const auto color = ToSdlColor(circle.color);
        const int first = static_cast<int>(vertices_.size());
        vertices_.push_back({ center, color, {} });
        for (int i = 0; i < segments; i++)
        {
            const float angle = 2.0f * std::numbers::pi_v<float> * static_cast<float>(i) / static_cast<float>(segments);
            vertices_.push_back({ { center.x + radius * std::cos(angle), center.y + radius * std::sin(angle) }, color,
                {} });
            indices_.push_back(first);
            indices_.push_back(first + 1 + i);
            indices_.push_back(first + 1 + (i + 1) % segments);
        }
    }
    if (!vertices_.empty() &&
        !SDL_RenderGeometry(renderer_, nullptr, vertices_.data(), static_cast<int>(vertices_.size()), indices_.data(),
            static_cast<int>(indices_.size())))
    {
        SDL_Log("SDL_RenderGeometry: %s", SDL_GetError());
    }
}

void SdlSampleDrawer::RenderBoxes(const DrawList& drawList, const Viewport& viewport)
{
    // Consecutive boxes of the same color go in one SDL_RenderFillRects
    std::size_t begin = 0;
    while (begin < drawList.boxes.size())
    {
        const auto color = drawList.boxes[begin].color;
        rects_.clear();
        std::size_t end = begin;
        for (; end < drawList.boxes.size() && drawList.boxes[end].color == color; ++end)
        {
            const auto& box = drawList.boxes[end];
            const auto topLeft = box.center - box.halfSize;
            rects_.push_back({ float{ topLeft.x } * viewport.scale + viewport.offsetX,
                float{ topLeft.y } * viewport.scale + viewport.offsetY,
                float{ box.halfSize.x } * 2.0f * viewport.scale, float{ box.halfSize.y } * 2.0f * viewport.scale });
        }
        SetDrawColor(color);
        SDL_RenderFillRects(renderer_, rects_.data(), static_cast<int>(rects_.size()));
        begin = end;
    }
}

void SdlSampleDrawer::RenderLines(const DrawList& drawList, const Viewport& viewport)
{
    // SDL_RenderLines draws a connected polyline, so independent segments are drawn one by one, grouped by color
    ColorId currentColor = sample_color::EMPTY;
    for (const auto& line : drawList.lines)
    {
        if (line.color != currentColor)
        {
            SetDrawColor(line.color);
            currentColor = line.color;
        }
        SDL_RenderLine(renderer_, float{ line.a.x } * viewport.scale + viewport.offsetX,
            float{ line.a.y } * viewport.scale + viewport.offsetY, float{ line.b.x } * viewport.scale + viewport.offsetX,
            float{ line.b.y } * viewport.scale + viewport.offsetY);
    }
}

void SdlSampleDrawer::SetDrawColor(ColorId color)
{
    const auto sdlColor = ToSdlColor(color);
    SDL_SetRenderDrawColorFloat(renderer_, sdlColor.r, sdlColor.g, sdlColor.b, sdlColor.a);
}

} // namespace neko
