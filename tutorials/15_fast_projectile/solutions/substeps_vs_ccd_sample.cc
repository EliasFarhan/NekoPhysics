#include "tutorials/15_fast_projectile/solutions/substeps_vs_ccd_sample.h"

#include "tutorials/15_fast_projectile/fast_projectile_sample.h"

namespace neko::tutorial {
namespace {
constexpr Scalar kMuzzleX = Ratio(3, 5);
constexpr Scalar kMuzzleSpread = Ratio(1, 2);
constexpr Scalar kWallX{5};
constexpr Vec2f kWallHalfSize{Ratio(1, 10), Ratio(1, 2)};
constexpr std::array<Scalar, 2> kLaneHeights{Ratio(24, 5), Ratio(12, 5)};
}  // namespace

int SubstepsVsCcdSample::needed_substeps() const {
  const Scalar step_length =
      parameters_[kSpeed].ToScalar() * Sample::kFixedDeltaTime;
  const Scalar caught = kShotRadius + kWallHalfSize.x();
  int substeps = 1;
  while (step_length > caught * static_cast<Scalar>(substeps)) {
    ++substeps;
  }
  return substeps;
}

std::uint32_t SubstepsVsCcdSample::GenerateChecksum() const {
  const std::uint32_t bullet = bullet_world_.GenerateChecksum();
  return substep_world_.has_value()
             ? substep_world_->GenerateChecksum() ^ bullet
             : bullet;
}

void SubstepsVsCcdSample::Begin(const SampleContext& context) {
  const int substeps = parameters_[kSubsteps].value;
  substep_world_.emplace(
      PhysicsWorldConfig{.gravity = Vec2f{Scalar{0}, -Ratio(981, 100)},
                         .dt = kFixedDeltaTime / static_cast<Scalar>(substeps),
                         .max_bodies = 4,
                         .max_colliders = 4,
                         .max_touching_pairs = 4,
                         .max_broadphase_pairs = 4,
                         .max_joints = 0,
                         .max_passing_pairs = 0});
  const Vec2f origin = context.world_bounds.min_bound();
  muzzle_x_ = origin.x() + kMuzzleX;
  wall_x_ = origin.x() + kWallX;
  for (std::size_t index = 0; index < 2; ++index) {
    lane_y_[index] = origin.y() + kLaneHeights[index];
    walls_[index] =
        AddWall(world(index), {wall_x_, lane_y_[index]}, kWallHalfSize);
    shots_[index] =
        AddShot(world(index), {muzzle_x_, lane_y_[index]}, index == 1, false);
  }
  FireBoth();
  UpdateReadouts();
}

void SubstepsVsCcdSample::FireBoth() {
  if (fired_ > 0) {
    for (std::size_t index = 0; index < 2; ++index) {
      if (world(index).ColliderCenter(shots_[index]).x() < wall_x_) {
        ++hits_[index];
      } else {
        ++misses_[index];
      }
    }
  }
  const Scalar start = muzzle_x_ + rng_.Range(Scalar{0}, kMuzzleSpread);
  for (std::size_t index = 0; index < 2; ++index) {
    Fire(world(index), shots_[index], {start, lane_y_[index]},
         {parameters_[kSpeed].ToScalar(), Scalar{0}});
  }
  ++fired_;
}

void SubstepsVsCcdSample::FixedUpdate() {
  if (step_ > 0 && step_ % kFirePeriod == 0) {
    FireBoth();
  }
  for (int substep = 0; substep < parameters_[kSubsteps].value; ++substep) {
    substep_world_->Step();
  }
  bullet_world_.Step();
  ++step_;
  UpdateReadouts();
}

void SubstepsVsCcdSample::UpdateReadouts() {
  readouts_[0].value = hits_[0];
  readouts_[1].value = misses_[0];
  readouts_[2].value = hits_[1];
  readouts_[3].value = misses_[1];
  readouts_[4].value = needed_substeps();
  readouts_[5].value = parameters_[kSubsteps].value;
  readouts_[6].value = 1;
  for (auto& readout : readouts_) {
    readout.is_valid = true;
  }
}

void SubstepsVsCcdSample::Draw(DrawList& draw_list) const {
  constexpr std::array<ColorId, 2> colors{sample_color::kOrange,
                                          sample_color::kSky};
  for (std::size_t index = 0; index < 2; ++index) {
    const PhysicsWorld& shown = index == 0 ? *substep_world_ : bullet_world_;
    draw_list.boxes.push_back({shown.ColliderCenter(walls_[index]),
                               kWallHalfSize, sample_color::kGrey});
    draw_list.circles.push_back(
        {shown.ColliderCenter(shots_[index]), kShotRadius, colors[index]});
  }
}

void SubstepsVsCcdSample::End() {
  substep_world_.reset();
  bullet_world_.Clear();
  rng_ = SampleRng{15};
  hits_ = {};
  misses_ = {};
  fired_ = 0;
  step_ = 0;
  for (auto& readout : readouts_) {
    readout.is_valid = false;
  }
}

}  // namespace neko::tutorial
