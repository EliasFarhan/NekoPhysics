#include "tutorials/15_fast_projectile/solutions/one_way_bullet_sample.h"

#include "neko/math/func.h"

#include "tutorials/15_fast_projectile/fast_projectile_sample.h"

namespace neko::tutorial {
namespace {
constexpr std::array<Scalar, 2> kColumnX{Ratio(16, 5), Ratio(48, 5)};
constexpr Scalar kLaunchY = Ratio(3, 5);
constexpr Scalar kLaunchSpread = Ratio(1, 2);
constexpr Scalar kPlatformY = Ratio(13, 5);
constexpr Vec2f kPlatformHalfSize{Scalar{1}, Ratio(1, 20)};
constexpr Scalar kBumperBottom{6};
constexpr Vec2f kBumperHalfSize{Scalar{1}, Ratio(3, 2)};
// Slower than this at the end of a firing: resting
constexpr Scalar kRestSpeed = Ratio(1, 2);
}  // namespace

void OneWayBulletSample::Begin(const SampleContext& context) {
  const Vec2f origin = context.world_bounds.min_bound();
  launch_y_ = origin.y() + kLaunchY;
  platform_top_ = origin.y() + kPlatformY + kPlatformHalfSize.y();
  bumper_bottom_ = origin.y() + kBumperBottom;
  for (std::size_t column = 0; column < 2; ++column) {
    column_x_[column] = origin.x() + kColumnX[column];
    platforms_[column] =
        AddWall(world_, {column_x_[column], origin.y() + kPlatformY},
                kPlatformHalfSize);
    world_.collider(platforms_[column]).is_one_way = true;
    bumpers_[column] = AddWall(
        world_, {column_x_[column], bumper_bottom_ + kBumperHalfSize.y()},
        kBumperHalfSize);
    world_.collider(bumpers_[column]).restitution = Scalar{1};
    shots_[column] = AddShot(world_, {}, column == 0, false);
  }
  FireBoth();
  UpdateReadouts();
}

void OneWayBulletSample::FireBoth() {
  if (fired_ > 0) {
    for (std::size_t column = 0; column < 2; ++column) {
      const Collider& shot = world_.collider(shots_[column]);
      const Scalar y = world_.ColliderCenter(shots_[column]).y();
      const Scalar speed = world_.body(shot.body_index).velocity.Length();
      if (is_up_[column]) {
        ++went_up_[column];
      }
      if (y > platform_top_ && y < bumper_bottom_ && speed < kRestSpeed) {
        ++landed_[column];
      }
    }
  }
  const Scalar start = launch_y_ + rng_.Range(Scalar{0}, kLaunchSpread);
  for (std::size_t column = 0; column < 2; ++column) {
    Fire(world_, shots_[column], {column_x_[column], start},
         {Scalar{0}, parameters_[kSpeed].ToScalar()});
    is_up_[column] = false;
  }
  ++fired_;
}

void OneWayBulletSample::FixedUpdate() {
  if (step_ > 0 && step_ % kFirePeriod == 0) {
    FireBoth();
  }
  world_.Step();
  ++step_;
  for (std::size_t column = 0; column < 2; ++column) {
    if (world_.ColliderCenter(shots_[column]).y() >
        platform_top_ + kShotRadius) {
      is_up_[column] = true;
    }
  }
  UpdateReadouts();
}

void OneWayBulletSample::UpdateReadouts() {
  readouts_[0].value = firings();
  readouts_[1].value = went_up_[0];
  readouts_[2].value = went_up_[1];
  readouts_[3].value = landed_[0];
  readouts_[4].value = landed_[1];
  for (auto& readout : readouts_) {
    readout.is_valid = true;
  }
}

void OneWayBulletSample::Draw(DrawList& draw_list) const {
  constexpr std::array<ColorId, 2> shot_colors{sample_color::kSky,
                                               sample_color::kOrange};
  for (std::size_t column = 0; column < 2; ++column) {
    draw_list.boxes.push_back({world_.ColliderCenter(platforms_[column]),
                               kPlatformHalfSize, sample_color::kTeal});
    draw_list.boxes.push_back({world_.ColliderCenter(bumpers_[column]),
                               kBumperHalfSize, sample_color::kGrey});
    draw_list.circles.push_back({world_.ColliderCenter(shots_[column]),
                                 kShotRadius, shot_colors[column]});
  }
}

void OneWayBulletSample::End() {
  world_.Clear();
  rng_ = SampleRng{15};
  is_up_ = {};
  went_up_ = {};
  landed_ = {};
  fired_ = 0;
  step_ = 0;
  for (auto& readout : readouts_) {
    readout.is_valid = false;
  }
}

}  // namespace neko::tutorial
