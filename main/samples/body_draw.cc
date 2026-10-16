#include "main/samples/body_draw.h"

#include <array>

#include "neko/physics/physics.h"

namespace neko {

void DrawOrientation(const Body& body, Scalar length, ColorId color,
                     DrawList& draw_list) {
  draw_list.lines.push_back(
      {body.position, body.position + body.rotation.x_axis() * length, color});
}

void DrawVelocity(const Body& body, ColorId color, DrawList& draw_list) {
  draw_list.lines.push_back({body.position,
                             body.position + body.velocity * kVelocityDrawTime,
                             color});
}

void DrawBoxOutline(Vec2f center, Vec2f half_size, const Rotation& rotation,
                    ColorId color, DrawList& draw_list) {
  // Corners counter-clockwise from the bottom left, in the box's frame
  const std::array<Vec2f, 4> corners = {{
      {-half_size.x(), -half_size.y()},
      {half_size.x(), -half_size.y()},
      {half_size.x(), half_size.y()},
      {-half_size.x(), half_size.y()},
  }};
  for (std::size_t i = 0; i < corners.size(); ++i) {
    const Vec2f a = center + rotation.Rotate(corners[i]);
    const Vec2f b = center + rotation.Rotate(corners[(i + 1) % corners.size()]);
    draw_list.lines.push_back({a, b, color});
  }
}

}  // namespace neko
