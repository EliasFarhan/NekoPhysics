// Gameplay forces and impulses (M2): force against impulse across time
// steps, mass dependence, force clearing, gravity scale and replay.
#include <cstdint>
#include <type_traits>

#include <gtest/gtest.h>

#include "neko/physics/physics.h"

namespace {

using neko::BodyIndex;
using neko::Scalar;
using neko::Vec2f;

template <typename T>
float ToFloat(T value) {
  if constexpr (std::is_same_v<T, float>) {
    return value;
  } else {
    return static_cast<float>(value);
  }
}

// A free body in a world without gravity, stepped at the given rate
struct FreeBody {
  explicit FreeBody(int steps_per_second, Scalar inverse_mass = Scalar{1})
      : world({.gravity = Vec2f::Zero(),
               .dt = Scalar{1} / static_cast<Scalar>(steps_per_second)}) {
    body = world.AddBody();
    world.body(body).inverse_mass = inverse_mass;
  }
  neko::PhysicsWorld world;
  BodyIndex body;
};

// A force held for one second changes the velocity by F / m whatever the
// time step
TEST(Force, HeldForOneSecondGivesForceOverMass) {
  for (const int rate : {30, 60, 120}) {
    FreeBody free{rate};
    for (int step = 0; step < rate; ++step) {
      free.world.ApplyForce(free.body, {Scalar{3}, Scalar{0}});
      free.world.Step();
    }
    // dt is rounded to 1/65536 with Fixed16: rate steps last a hair under 1 s
    EXPECT_NEAR(ToFloat(free.world.body(free.body).velocity.x()), 3.0f, 3e-3f)
        << rate;
  }
}

// An impulse once changes the velocity by J / m whatever the time step;
// applied every step instead, its effect grows with the step rate
TEST(Impulse, OnceIsRateIndependentEveryStepIsNot) {
  for (const int rate : {30, 60, 120}) {
    FreeBody once{rate};
    FreeBody every_step{rate};
    once.world.ApplyImpulse(once.body, {Scalar{2}, Scalar{0}});
    for (int step = 0; step < rate; ++step) {
      every_step.world.ApplyImpulse(every_step.body, {Scalar{2}, Scalar{0}});
      once.world.Step();
      every_step.world.Step();
    }
    EXPECT_EQ(ToFloat(once.world.body(once.body).velocity.x()), 2.0f) << rate;
    EXPECT_EQ(ToFloat(every_step.world.body(every_step.body).velocity.x()),
              2.0f * static_cast<float>(rate))
        << rate;
  }
}

TEST(Impulse, HeavierBodyGetsLess) {
  FreeBody light{60, Scalar{1}};
  FreeBody heavy{60, Scalar{1} / Scalar{4}};
  light.world.ApplyImpulse(light.body, {Scalar{0}, Scalar{2}});
  heavy.world.ApplyImpulse(heavy.body, {Scalar{0}, Scalar{2}});
  EXPECT_EQ(ToFloat(light.world.body(light.body).velocity.y()), 2.0f);
  EXPECT_EQ(ToFloat(heavy.world.body(heavy.body).velocity.y()), 0.5f);
  // Through the center: the same as ApplyImpulseAtPoint at the position
  FreeBody at_point{60, Scalar{1} / Scalar{4}};
  at_point.world.ApplyImpulseAtPoint(
      at_point.body, {Scalar{0}, Scalar{2}},
      at_point.world.body(at_point.body).position);
  EXPECT_EQ(at_point.world.body(at_point.body).velocity,
            heavy.world.body(heavy.body).velocity);
}

TEST(Impulse, KinematicAndStaticIgnoreIt) {
  FreeBody free{60};
  for (const auto type :
       {neko::BodyType::kKinematic, neko::BodyType::kStatic}) {
    free.world.body(free.body).type = type;
    free.world.ApplyImpulse(free.body, {Scalar{5}, Scalar{5}});
    EXPECT_EQ(free.world.body(free.body).velocity, Vec2f::Zero());
  }
}

// A force lasts one step: the world clears it
TEST(Force, ClearedAfterEachStep) {
  FreeBody free{60};
  free.world.ApplyForce(free.body, {Scalar{6}, Scalar{0}});
  free.world.ApplyForce(free.body, {Scalar{0}, Scalar{6}});  // they add up
  EXPECT_EQ(free.world.body(free.body).force, (Vec2f{Scalar{6}, Scalar{6}}));
  free.world.Step();
  EXPECT_EQ(free.world.body(free.body).force, Vec2f::Zero());
  const Vec2f after_one = free.world.body(free.body).velocity;
  free.world.Step();
  EXPECT_EQ(free.world.body(free.body).velocity, after_one);
}

// Gravity scale: 0 floats, 2 falls twice as fast, -1 falls up; the scale is
// the same for every mass, unlike a force
TEST(GravityScale, ScalesTheFall) {
  struct Case {
    Scalar scale;
    float expected;
  };
  for (const Case& c : {Case{Scalar{0}, 0.0f}, Case{Scalar{1}, -9.81f},
                        Case{Scalar{2}, -19.62f}, Case{Scalar{-1}, 9.81f}}) {
    for (const Scalar inverse_mass : {Scalar{1}, Scalar{1} / Scalar{10}}) {
      neko::PhysicsWorld world;
      const BodyIndex body = world.AddBody();
      world.body(body).inverse_mass = inverse_mass;
      world.body(body).gravity_scale = c.scale;
      for (int step = 0; step < 60; ++step) {
        world.Step();
      }
      EXPECT_NEAR(ToFloat(world.body(body).velocity.y()), c.expected, 0.02f)
          << ToFloat(c.scale);
    }
  }
}

// The default scale leaves the checksum alone; another one is state
TEST(GravityScale, ChecksumAndCopy) {
  neko::PhysicsWorld world;
  const BodyIndex body = world.AddBody();
  const std::uint32_t unscaled = world.GenerateChecksum();
  world.body(body).gravity_scale = Scalar{0};
  EXPECT_NE(world.GenerateChecksum(), unscaled);
  neko::PhysicsWorld copy;
  copy.CopyFrom(world);
  EXPECT_EQ(copy.body(body).gravity_scale, Scalar{0});
  EXPECT_EQ(copy.GenerateChecksum(), world.GenerateChecksum());
}

// Rollback: replaying the same forces and impulses from a snapshot gives
// the same world, bit for bit
TEST(Force, ReplayFromSnapshot) {
  neko::PhysicsWorld world;
  const BodyIndex body = world.AddBody();
  world.body(body).gravity_scale = Scalar{1} / Scalar{2};
  const auto input = [&](neko::PhysicsWorld& w, int step) {
    if (step % 3 == 0) {
      w.ApplyForce(body, {Scalar{4}, Scalar{12}});
    }
    if (step == 7) {
      w.ApplyImpulse(body, {Scalar{-3}, Scalar{1}});
    }
  };
  for (int step = 0; step < 10; ++step) {
    input(world, step);
    world.Step();
  }
  neko::PhysicsWorld snapshot;
  snapshot.CopyFrom(world);
  for (int step = 10; step < 40; ++step) {
    input(world, step);
    world.Step();
  }
  const std::uint32_t expected = world.GenerateChecksum();
  world.CopyFrom(snapshot);
  for (int step = 10; step < 40; ++step) {
    input(world, step);
    world.Step();
  }
  EXPECT_EQ(world.GenerateChecksum(), expected);
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
