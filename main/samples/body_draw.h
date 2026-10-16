#ifndef MAIN_SAMPLES_BODY_DRAW_H_
#define MAIN_SAMPLES_BODY_DRAW_H_

#include "neko/math/const.h"
#include "neko/math/rotation.h"
#include "neko/math/vec2.h"

#include "main/samples/draw_list.h"
#include "main/samples/sample_config.h"

namespace neko {

struct Body;

/// Time over which @ref DrawVelocity draws a velocity, in seconds: the line
/// is the distance the body covers in 0.1 s.
inline constexpr Scalar kVelocityDrawTime = Ratio(1, 10);

/// Draws a body's orientation: a line from its position along its local x
/// axis (@ref Rotation::x_axis).
///
/// The debug view of rotation. A body that doesn't turn keeps a line
/// pointing right (+x); a positive angle turns it counter-clockwise.
///
/// @param body The body to show.
/// @param length Length of the line, in meters.
/// @param color Line color.
/// @param draw_list The list to add the line to.
void DrawOrientation(const Body& body, Scalar length, ColorId color,
                     DrawList& draw_list);

/// Draws a body's velocity: a line from its position to where it will be in
/// @ref kVelocityDrawTime seconds at its current velocity.
///
/// @param body The body to show.
/// @param color Line color.
/// @param draw_list The list to add the line to.
void DrawVelocity(const Body& body, ColorId color, DrawList& draw_list);

/// Draws the outline of a box turned by `rotation`, as four lines.
///
/// @ref DrawBox only draws axis-aligned boxes; use this for a box that turns
/// with its body.
///
/// @param center Center of the box, in meters.
/// @param half_size Half width and half height in the box's own frame.
/// @param rotation Orientation of the box (@ref Rotation).
/// @param color Line color.
/// @param draw_list The list to add the lines to.
void DrawBoxOutline(Vec2f center, Vec2f half_size, const Rotation& rotation,
                    ColorId color, DrawList& draw_list);

}  // namespace neko

#endif  // MAIN_SAMPLES_BODY_DRAW_H_
