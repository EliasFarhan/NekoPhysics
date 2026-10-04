#ifndef MAIN_SDL_SDL_SAMPLE_DRAWER_H_
#define MAIN_SDL_SDL_SAMPLE_DRAWER_H_

#include <vector>

#include <SDL3/SDL_render.h>

#include "main/samples/draw_list.h"

namespace neko {

// Renders a DrawList with an SDL renderer: the camera is fitted into the window
// with letterboxing, circles are tessellated into one geometry batch, boxes and
// lines are batched by color.
class SdlSampleDrawer final : public SampleDrawer {
 public:
  explicit SdlSampleDrawer(SDL_Renderer* renderer) : renderer_(renderer) {}
  void Render(const DrawList& draw_list) override;

 private:
  struct Viewport {
    float scale = 1.0f;
    float offset_x = 0.0f;
    float offset_y = 0.0f;
  };
  [[nodiscard]] Viewport FitCamera(const Aabbf& camera) const;
  void RenderCircles(const DrawList& draw_list, const Viewport& viewport);
  void RenderBoxes(const DrawList& draw_list, const Viewport& viewport);
  void RenderLines(const DrawList& draw_list, const Viewport& viewport);
  void SetDrawColor(ColorId color);

  SDL_Renderer* renderer_;
  std::vector<SDL_Vertex> vertices_;
  std::vector<int> indices_;
  std::vector<SDL_FRect> rects_;
};

}  // namespace neko

#endif  // MAIN_SDL_SDL_SAMPLE_DRAWER_H_
