#include "tutorials/tutorial_catalog.h"

#include <array>

#include "tutorials/00_bounce/bounce_sample.h"
#include "tutorials/00_bounce/solutions/bounces_to_rest_sample.h"
#include "tutorials/00_bounce/solutions/explicit_euler_sample.h"
#include "tutorials/00_bounce/solutions/head_on_sample.h"
#include "tutorials/00_bounce/solutions/tunneling_sample.h"
#include "tutorials/01_rotation/rotation_sample.h"
#include "tutorials/01_rotation/solutions/disk_sample.h"
#include "tutorials/01_rotation/solutions/dumbbell_sample.h"
#include "tutorials/01_rotation/solutions/half_step_sample.h"
#include "tutorials/01_rotation/solutions/single_force_sample.h"
#include "tutorials/02_rotated_box/rotated_box_sample.h"
#include "tutorials/02_rotated_box/solutions/box_on_ramp_sample.h"
#include "tutorials/02_rotated_box/solutions/corner_sample.h"
#include "tutorials/02_rotated_box/solutions/tight_bounds_sample.h"
#include "tutorials/02_rotated_box/solutions/two_ramps_sample.h"
#include "tutorials/03_contact_manifold/contact_manifold_sample.h"
#include "tutorials/03_contact_manifold/solutions/box_on_tilted_box_sample.h"
#include "tutorials/03_contact_manifold/solutions/narrow_pillar_sample.h"
#include "tutorials/03_contact_manifold/solutions/points_over_a_turn_sample.h"
#include "tutorials/03_contact_manifold/solutions/two_aabbs_sample.h"
#include "tutorials/04_off_center_impact/off_center_impact_sample.h"
#include "tutorials/04_off_center_impact/solutions/ball_hits_box_sample.h"
#include "tutorials/04_off_center_impact/solutions/energy_budget_sample.h"
#include "tutorials/04_off_center_impact/solutions/fastest_spin_sample.h"
#include "tutorials/04_off_center_impact/solutions/passes_sample.h"
#include "tutorials/05_friction/friction_sample.h"
#include "tutorials/05_friction/solutions/mixed_surfaces_sample.h"
#include "tutorials/05_friction/solutions/push_to_start_sample.h"
#include "tutorials/05_friction/solutions/rolling_disk_sample.h"
#include "tutorials/05_friction/solutions/stopping_distance_sample.h"
#include "tutorials/06_stacking/solutions/heavy_on_light_sample.h"
#include "tutorials/06_stacking/solutions/impulse_per_box_sample.h"
#include "tutorials/06_stacking/solutions/pyramid_sample.h"
#include "tutorials/06_stacking/solutions/two_boxes_by_hand_sample.h"
#include "tutorials/06_stacking/stacking_sample.h"
#include "tutorials/07_pendulum/pendulum_sample.h"
#include "tutorials/07_pendulum/solutions/energy_sample.h"
#include "tutorials/07_pendulum/solutions/physical_pendulum_sample.h"
#include "tutorials/07_pendulum/solutions/tension_sample.h"
#include "tutorials/07_pendulum/solutions/where_slack_sample.h"
#include "tutorials/08_chain/chain_sample.h"
#include "tutorials/08_chain/solutions/hertz_sweep_sample.h"
#include "tutorials/08_chain/solutions/joint_order_sample.h"
#include "tutorials/08_chain/solutions/one_step_by_hand_sample.h"
#include "tutorials/08_chain/solutions/per_link_stretch_sample.h"
#include "tutorials/09_spring/solutions/critical_damping_sample.h"
#include "tutorials/09_spring/solutions/soft_vs_explicit_sample.h"
#include "tutorials/09_spring/solutions/stability_limit_sample.h"
#include "tutorials/09_spring/solutions/substeps_sample.h"
#include "tutorials/09_spring/spring_sample.h"
#include "tutorials/10_explosion/explosion_sample.h"
#include "tutorials/10_explosion/solutions/laser_bounce_sample.h"
#include "tutorials/10_explosion/solutions/long_laser_sample.h"
#include "tutorials/10_explosion/solutions/one_hit_sample.h"
#include "tutorials/10_explosion/solutions/reach_sample.h"
#include "tutorials/11_jetpack/jetpack_sample.h"
#include "tutorials/11_jetpack/solutions/decaying_thrust_sample.h"
#include "tutorials/11_jetpack/solutions/hover_sample.h"
#include "tutorials/11_jetpack/solutions/step_rate_sample.h"
#include "tutorials/11_jetpack/solutions/terminal_velocity_sample.h"
#include "tutorials/12_aiming/aiming_sample.h"
#include "tutorials/12_aiming/solutions/error_table_sample.h"
#include "tutorials/12_aiming/solutions/high_arc_sample.h"
#include "tutorials/12_aiming/solutions/moving_target_sample.h"
#include "tutorials/12_aiming/solutions/out_of_range_sample.h"
#include "tutorials/13_one_way/one_way_sample.h"
#include "tutorials/13_one_way/solutions/arc_sweep_sample.h"
#include "tutorials/13_one_way/solutions/drop_through_sample.h"
#include "tutorials/13_one_way/solutions/ghost_timer_sample.h"
#include "tutorials/13_one_way/solutions/stacked_one_way_sample.h"
#include "tutorials/14_moving_platform/moving_platform_sample.h"
#include "tutorials/14_moving_platform/solutions/lag_measure_sample.h"
#include "tutorials/14_moving_platform/solutions/rider_carry_sample.h"
#include "tutorials/14_moving_platform/solutions/teleport_warm_start_sample.h"
#include "tutorials/14_moving_platform/solutions/wrap_events_sample.h"
#include "tutorials/15_fast_projectile/fast_projectile_sample.h"
#include "tutorials/15_fast_projectile/solutions/one_way_bullet_sample.h"
#include "tutorials/15_fast_projectile/solutions/speed_threshold_sample.h"
#include "tutorials/15_fast_projectile/solutions/substeps_vs_ccd_sample.h"
#include "tutorials/15_fast_projectile/solutions/trigger_hits_sample.h"

namespace neko::tutorial {
namespace {
template <typename T>
std::unique_ptr<Sample> Create() {
  return std::make_unique<T>();
}

const std::array<TutorialEntry, 80> kTutorials{{
    {"00_bounce/demo", "00 bounce", "00 Bounce", true, &Create<BounceSample>},
    {"00_bounce/bounces_to_rest", "00 bounce", "00 Bounces to rest", false,
     &Create<BouncesToRestSample>},
    {"00_bounce/explicit_euler", "00 bounce", "00 Explicit Euler", false,
     &Create<ExplicitEulerSample>},
    {"00_bounce/head_on", "00 bounce", "00 Head on", false,
     &Create<HeadOnSample>},
    {"00_bounce/tunneling", "00 bounce", "00 Tunneling", false,
     &Create<TunnelingSample>},
    {"01_rotation/demo", "01 rotation", "01 Rotation", true,
     &Create<RotationSample>},
    {"01_rotation/disk", "01 rotation", "01 Disk", false, &Create<DiskSample>},
    {"01_rotation/dumbbell", "01 rotation", "01 Dumbbell", false,
     &Create<DumbbellSample>},
    {"01_rotation/half_step", "01 rotation", "01 Half step", false,
     &Create<HalfStepSample>},
    {"01_rotation/single_force", "01 rotation", "01 Single force", false,
     &Create<SingleForceSample>},
    {"02_rotated_box/demo", "02 rotated box", "02 Rotated box", true,
     &Create<RotatedBoxSample>},
    {"02_rotated_box/box_on_ramp", "02 rotated box", "02 Box on ramp", false,
     &Create<BoxOnRampSample>},
    {"02_rotated_box/corner", "02 rotated box", "02 Corner", false,
     &Create<CornerSample>},
    {"02_rotated_box/tight_bounds", "02 rotated box", "02 Tight bounds", false,
     &Create<TightBoundsSample>},
    {"02_rotated_box/two_ramps", "02 rotated box", "02 Two ramps", false,
     &Create<TwoRampsSample>},
    {"03_contact_manifold/demo", "03 contact manifold", "03 Contact manifold",
     true, &Create<ContactManifoldSample>},
    {"03_contact_manifold/box_on_tilted_box", "03 contact manifold",
     "03 Box on a tilted box", false, &Create<BoxOnTiltedBoxSample>},
    {"03_contact_manifold/narrow_pillar", "03 contact manifold",
     "03 Narrow pillar", false, &Create<NarrowPillarSample>},
    {"03_contact_manifold/points_over_a_turn", "03 contact manifold",
     "03 Points over a turn", false, &Create<PointsOverATurnSample>},
    {"03_contact_manifold/two_aabbs", "03 contact manifold", "03 Two AABBs",
     false, &Create<TwoAabbsSample>},
    {"04_off_center_impact/demo", "04 off center impact",
     "04 Off-center impact", true, &Create<OffCenterImpactSample>},
    {"04_off_center_impact/ball_hits_box", "04 off center impact",
     "04 Ball hits box", false, &Create<BallHitsBoxSample>},
    {"04_off_center_impact/energy_budget", "04 off center impact",
     "04 Energy budget", false, &Create<EnergyBudgetSample>},
    {"04_off_center_impact/fastest_spin", "04 off center impact",
     "04 Fastest spin", false, &Create<FastestSpinSample>},
    {"04_off_center_impact/passes", "04 off center impact",
     "04 One pass vs eight", false, &Create<PassesSample>},
    {"05_friction/demo", "05 friction", "05 Friction", true,
     &Create<FrictionSample>},
    {"05_friction/mixed_surfaces", "05 friction", "05 Mixed surfaces", false,
     &Create<MixedSurfacesSample>},
    {"05_friction/push_to_start", "05 friction", "05 Push to start", false,
     &Create<PushToStartSample>},
    {"05_friction/rolling_disk", "05 friction", "05 Rolling disk", false,
     &Create<RollingDiskSample>},
    {"05_friction/stopping_distance", "05 friction", "05 Stopping distance",
     false, &Create<StoppingDistanceSample>},
    {"06_stacking/demo", "06 stacking", "06 Stacking", true,
     &Create<StackingSample>},
    {"06_stacking/heavy_on_light", "06 stacking", "06 Heavy on light", false,
     &Create<HeavyOnLightSample>},
    {"06_stacking/impulse_per_box", "06 stacking", "06 Impulse per box", false,
     &Create<ImpulsePerBoxSample>},
    {"06_stacking/pyramid", "06 stacking", "06 Pyramid", false,
     &Create<PyramidSample>},
    {"06_stacking/two_boxes_by_hand", "06 stacking", "06 Two boxes by hand",
     false, &Create<TwoBoxesByHandSample>},
    {"07_pendulum/demo", "07 pendulum", "07 Pendulum", true,
     &Create<PendulumSample>},
    {"07_pendulum/energy", "07 pendulum", "07 Energy", false,
     &Create<EnergySample>},
    {"07_pendulum/physical_pendulum", "07 pendulum", "07 Physical pendulum",
     false, &Create<PhysicalPendulumSample>},
    {"07_pendulum/tension", "07 pendulum", "07 Tension", false,
     &Create<TensionSample>},
    {"07_pendulum/where_slack", "07 pendulum", "07 Where slack", false,
     &Create<WhereSlackSample>},
    {"08_chain/demo", "08 chain", "08 Chain", true, &Create<ChainSample>},
    {"08_chain/hertz_sweep", "08 chain", "08 Hertz sweep", false,
     &Create<HertzSweepSample>},
    {"08_chain/joint_order", "08 chain", "08 Joint order", false,
     &Create<JointOrderSample>},
    {"08_chain/one_step_by_hand", "08 chain", "08 One step by hand", false,
     &Create<OneStepByHandSample>},
    {"08_chain/per_link_stretch", "08 chain", "08 Per link stretch", false,
     &Create<PerLinkStretchSample>},
    {"09_spring/demo", "09 spring", "09 Spring", true, &Create<SpringSample>},
    {"09_spring/critical_damping", "09 spring", "09 Critical damping", false,
     &Create<CriticalDampingSample>},
    {"09_spring/soft_vs_explicit", "09 spring", "09 Soft vs explicit", false,
     &Create<SoftVsExplicitSample>},
    {"09_spring/stability_limit", "09 spring", "09 Stability limit", false,
     &Create<StabilityLimitSample>},
    {"09_spring/substeps", "09 spring", "09 Substeps", false,
     &Create<SubstepsSample>},
    {"10_explosion/demo", "10 explosion", "10 Explosion", true,
     &Create<ExplosionSample>},
    {"10_explosion/laser_bounce", "10 explosion", "10 Laser bounce", false,
     &Create<LaserBounceSample>},
    {"10_explosion/long_laser", "10 explosion", "10 Long laser", false,
     &Create<LongLaserSample>},
    {"10_explosion/one_hit", "10 explosion", "10 One hit per body", false,
     &Create<OneHitSample>},
    {"10_explosion/reach", "10 explosion", "10 Reach", false,
     &Create<ReachSample>},
    {"11_jetpack/demo", "11 jetpack", "11 Jetpack", true,
     &Create<JetpackSample>},
    {"11_jetpack/decaying_thrust", "11 jetpack", "11 Decaying thrust", false,
     &Create<DecayingThrustSample>},
    {"11_jetpack/hover", "11 jetpack", "11 Hover", false, &Create<HoverSample>},
    {"11_jetpack/step_rate", "11 jetpack", "11 Step rate", false,
     &Create<StepRateSample>},
    {"11_jetpack/terminal_velocity", "11 jetpack", "11 Terminal velocity",
     false, &Create<TerminalVelocitySample>},
    {"12_aiming/demo", "12 aiming", "12 Aiming", true, &Create<AimingSample>},
    {"12_aiming/error_table", "12 aiming", "12 Error table", false,
     &Create<ErrorTableSample>},
    {"12_aiming/high_arc", "12 aiming", "12 High arc", false,
     &Create<HighArcSample>},
    {"12_aiming/moving_target", "12 aiming", "12 Moving target", false,
     &Create<MovingTargetSample>},
    {"12_aiming/out_of_range", "12 aiming", "12 Out of range", false,
     &Create<OutOfRangeSample>},
    {"13_one_way/demo", "13 one way", "13 One-way", true,
     &Create<OneWaySample>},
    {"13_one_way/arc_sweep", "13 one way", "13 Arc sweep", false,
     &Create<ArcSweepSample>},
    {"13_one_way/drop_through", "13 one way", "13 Drop through", false,
     &Create<DropThroughSample>},
    {"13_one_way/ghost_timer", "13 one way", "13 Ghost timer", false,
     &Create<GhostTimerSample>},
    {"13_one_way/stacked_one_way", "13 one way", "13 Stacked one-way", false,
     &Create<StackedOneWaySample>},
    {"14_moving_platform/demo", "14 moving platform", "14 Moving platforms",
     true, &Create<MovingPlatformSample>},
    {"14_moving_platform/lag_measure", "14 moving platform", "14 Lag measure",
     false, &Create<LagMeasureSample>},
    {"14_moving_platform/rider_carry", "14 moving platform", "14 Rider carry",
     false, &Create<RiderCarrySample>},
    {"14_moving_platform/teleport_warm_start", "14 moving platform",
     "14 Teleport warm start", false, &Create<TeleportWarmStartSample>},
    {"14_moving_platform/wrap_events", "14 moving platform", "14 Wrap events",
     false, &Create<WrapEventsSample>},
    {"15_fast_projectile/demo", "15 fast projectile", "15 Fast projectiles",
     true, &Create<FastProjectileSample>},
    {"15_fast_projectile/one_way_bullet", "15 fast projectile",
     "15 One-way bullet", false, &Create<OneWayBulletSample>},
    {"15_fast_projectile/speed_threshold", "15 fast projectile",
     "15 Speed threshold", false, &Create<SpeedThresholdSample>},
    {"15_fast_projectile/substeps_vs_ccd", "15 fast projectile",
     "15 Substeps vs CCD", false, &Create<SubstepsVsCcdSample>},
    {"15_fast_projectile/trigger_hits", "15 fast projectile", "15 Trigger hits",
     false, &Create<TriggerHitsSample>},
}};
}  // namespace

std::span<const TutorialEntry> TutorialCatalog() { return kTutorials; }

std::size_t FindTutorial(std::string_view id) {
  for (std::size_t i = 0; i < kTutorials.size(); ++i) {
    if (id == kTutorials[i].id) return i;
  }
  return SampleRegistry::kNone;
}
}  // namespace neko::tutorial
