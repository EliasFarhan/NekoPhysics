// DS test ROM: DS == PC determinism of the falling pile scene, self-check of the hardware math hooks, Step timings
// and a per-phase breakdown of Step. Results go to the bottom screen and are kept in the g_report global so a
// host-side arm-none-eabi-gdb session (melonDS exposes a GDB stub, see [Instance0.Gdb] in melonDS.toml) can read
// exact values back after the TestsDone() breakpoint: melonDS does not implement the no$gba debug message port, and
// the on-screen console (32x24) scrolls past the early lines well before the run is done.
#include "neko/core/step_profile.h"
#include "neko/physics/physics.h"
#include "tests/scenes/falling_pile.h"

#include <nds.h>

#include <cinttypes>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <utility>

namespace
{

__attribute__((format(printf, 1, 2))) void Log(const char* format, ...)
{
    char line[96];
    va_list args;
    va_start(args, format);
    std::vsnprintf(line, sizeof(line), format, args);
    va_end(args);
    std::fputs(line, stdout);
    nocashMessage(line);
}

} // namespace

struct Timing
{
    std::uint64_t total = 0;
    std::uint32_t max = 0;
    int count = 0;

    void Add(std::uint32_t ticks)
    {
        total += ticks;
        max = ticks > max ? ticks : max;
        ++count;
    }
};

struct ChecksumSample
{
    int step = 0;
    std::uint32_t checksum = 0;
    std::uint32_t expected = 0;
};

struct PairCountSample
{
    int step = 0;
    std::uint16_t broadphasePairs = 0;
    std::uint16_t touchingPairs = 0;
};

/**
 * @brief Every number the vault report is built from, kept in one global struct so a host-side gdb session can
 * `print g_report` (or any field) once the ROM reaches the TestsDone() breakpoint.
 */
struct Report
{
    bool hooksOk = false;
    bool pileOk = false;
    ChecksumSample pileChecksums[6] = {};
    PairCountSample pilePairCounts[6] = {};
    Timing pileTiming{};

    std::uint32_t heavyChecksum = 0;
    PairCountSample heavyPairCounts[4] = {};
    Timing heavyTiming{};

    neko::StepPhaseTicks heavyPhaseTicks{};
    int heavyPhaseSteps = 0;
};

Report g_report;

namespace
{

std::uint32_t TimedStep(neko::PhysicsWorld& world, Timing& timing)
{
    cpuStartTiming(0);
    world.Step();
    const std::uint32_t ticks = cpuEndTiming();
    timing.Add(ticks);
    return ticks;
}

// cpuStartTiming counts at the bus clock (BUS_CLOCK, ~33.51 MHz); the ARM9 runs at twice that
void LogTiming(const char* name, const Timing& timing)
{
    const auto average = static_cast<std::uint32_t>(timing.total / static_cast<std::uint64_t>(timing.count));
    Log("%s avg %" PRIu32 "us max %" PRIu32 "us\n", name, timerTicks2usec(average), timerTicks2usec(timing.max));
    Log("  ticks avg %" PRIu32 " max %" PRIu32 "\n", average, timing.max);
}

PairCountSample SamplePairCounts(int step, const neko::PhysicsWorld& world)
{
    const PairCountSample sample{ step, static_cast<std::uint16_t>(world.broadphasePairCount()),
                                   static_cast<std::uint16_t>(world.touchingPairs().size()) };
    Log("  step %3d broadphase %3u touching %3u\n", sample.step, sample.broadphasePairs, sample.touchingPairs);
    return sample;
}

std::uint64_t Next(std::uint64_t& state)
{
    // xorshift64
    state ^= state << 13;
    state ^= state >> 7;
    state ^= state << 17;
    return state;
}

/**
 * @brief Compares the DS divider and sqrt hooks with the portable code on random and edge inputs
 */
bool CheckMathHooks()
{
    int failures = 0;
    const auto checkDiv = [&failures](std::int64_t num, std::int32_t den)
    {
        if (den == 0 || (den == -1 && num == INT64_MIN))
        {
            return;
        }
        if (neko::detail::Div64(num, den) != neko::detail::Div64Portable(num, den) && failures++ < 3)
        {
            Log("div %" PRId64 "/%" PRId32 " FAIL\n", num, den);
        }
    };
    int sqrtFailuresByBits[65] = {};
    const auto checkSqrt = [&failures, &sqrtFailuresByBits](std::uint64_t value)
    {
        const std::uint32_t hardware = neko::detail::ISqrt64(value);
        const std::uint32_t portable = neko::detail::ISqrt64Portable(value);
        if (hardware != portable)
        {
            ++sqrtFailuresByBits[64 - __builtin_clzll(value)];
            if (failures++ < 2)
            {
                Log("sqrt %" PRIx64 " hw %" PRIx32 " sw %" PRIx32 "\n", value, hardware, portable);
            }
        }
    };

    constexpr std::int32_t edgeDens[] = { 1, -1, 2, -2, 3, 7, 65536, -65536, INT32_MAX, INT32_MIN };
    constexpr std::int64_t edgeNums[] = { 0, 1, -1, 7, -7, INT32_MAX, INT32_MIN, std::int64_t{ INT32_MAX } << 16,
                                          std::int64_t{ INT32_MIN } << 16, INT64_MAX, INT64_MIN };
    for (const auto num : edgeNums)
    {
        for (const auto den : edgeDens)
        {
            checkDiv(num, den);
        }
        checkSqrt(static_cast<std::uint64_t>(num));
    }

    std::uint64_t state = 0x9E3779B97F4A7C15u;
    constexpr int samples = 20000;
    for (int i = 0; i < samples; ++i)
    {
        const std::uint64_t a = Next(state);
        const std::uint64_t b = Next(state);
        // Fixed16 numerators (raw << 16) and denominators, then full-range ones
        const auto raw = static_cast<std::int32_t>(static_cast<std::uint32_t>(a));
        const auto den = static_cast<std::int32_t>(static_cast<std::uint32_t>(b));
        const auto smallDen = static_cast<std::int32_t>(static_cast<std::int16_t>(static_cast<std::uint16_t>(b >> 32)));
        checkDiv(std::int64_t{ raw } * 65536, den);
        checkDiv(std::int64_t{ raw } * 65536, smallDen);
        checkDiv(static_cast<std::int64_t>(a), den);
        checkSqrt(a);
        checkSqrt(a >> (b & 63u));
        const auto root = static_cast<std::uint64_t>(static_cast<std::uint32_t>(b));
        checkSqrt(root * root);
        checkSqrt(root * root - 1u);
    }
    for (int bits = 0; bits <= 64; ++bits)
    {
        if (sqrtFailuresByBits[bits] != 0)
        {
            Log("sqrt %d-bit inputs: %d errors\n", bits, sqrtFailuresByBits[bits]);
        }
    }
    Log("math hooks: %s (%d errors)\n", failures == 0 ? "OK" : "FAIL", failures);
    return failures == 0;
}

bool RunFallingPile()
{
    neko::PhysicsWorld world{};
    neko::scenes::BuildFallingPile(world);
    bool ok = true;
    int step = 0;
    int sampleIndex = 0;
    for (const auto& [expectedStep, expected] : neko::scenes::FALLING_PILE_GOLDEN_CHECKSUMS)
    {
        while (step < expectedStep)
        {
            TimedStep(world, g_report.pileTiming);
            ++step;
        }
        const std::uint32_t checksum = world.GenerateChecksum();
        ok = ok && checksum == expected;
        Log("%3d %08" PRIx32 " %08" PRIx32 " %s\n", step, checksum, expected, checksum == expected ? "OK" : "FAIL");
        g_report.pileChecksums[sampleIndex] = { step, checksum, expected };
        g_report.pilePairCounts[sampleIndex] = SamplePairCounts(step, world);
        ++sampleIndex;
    }
    Log("pile: %s\n", ok ? "OK" : "FAIL");
    LogTiming("67", g_report.pileTiming);
    return ok;
}

/**
 * @brief 3 static boxes and 252 dynamic bodies, near the default capacity of 256 bodies
 */
void BuildHeavyPile(neko::PhysicsWorld& world)
{
    using neko::Scalar;
    using neko::scenes::Ratio;
    const auto addStaticBox = [&world](neko::Vec2f position, neko::Vec2f halfSize)
    {
        const auto body = world.AddBody();
        world.body(body).type = neko::BodyType::STATIC;
        world.body(body).position = position;
        const auto collider = world.AddAabbCollider(body);
        world.collider(collider).isTrigger = false;
        world.collider(collider).restitution = Ratio(1, 4);
        world.aabb(world.collider(collider).shapeIndex).halfSize = halfSize;
    };
    addStaticBox({ Scalar{ 0 }, Ratio(-1, 2) }, { Scalar{ 12 }, Ratio(1, 2) });
    addStaticBox({ Ratio(-25, 2), Scalar{ 10 } }, { Ratio(1, 2), Scalar{ 11 } });
    addStaticBox({ Ratio(25, 2), Scalar{ 10 } }, { Ratio(1, 2), Scalar{ 11 } });

    constexpr int dynamicCount = 252;
    constexpr int columns = 18;
    for (int i = 0; i < dynamicCount; ++i)
    {
        const auto body = world.AddBody();
        const int column = i % columns;
        const int row = i / columns;
        world.body(body).position = { Ratio(-102 + 12 * column + 3 * (row % 2), 10), Ratio(10 + 11 * row, 10) };
        world.body(body).velocity = { Ratio(7 * (i % 5 - 2), 10), static_cast<Scalar>(i % 3) };
        world.body(body).inverseMass = i % 4 == 0 ? Ratio(1, 2) : Scalar{ 1 };
        const bool isCircle = i % 3 != 0;
        const auto collider = isCircle ? world.AddCircleCollider(body) : world.AddAabbCollider(body);
        world.collider(collider).isTrigger = false;
        world.collider(collider).restitution = Ratio(i % 4, 10);
        if (isCircle)
        {
            world.circle(world.collider(collider).shapeIndex).radius = Ratio(6 + i % 3, 20);
        }
        else
        {
            world.aabb(world.collider(collider).shapeIndex).halfSize = { Ratio(7, 20), Ratio(5 + i % 2, 20) };
        }
    }
}

constexpr int HEAVY_STEPS = 600;

/**
 * @brief Timing and checksum only, with no profiling overhead: the numbers reported for the 8ms budget
 */
void RunHeavyPile()
{
    neko::PhysicsWorld world{};
    BuildHeavyPile(world);

    int sampleIndex = 0;
    for (int step = 0; step < HEAVY_STEPS; ++step)
    {
        TimedStep(world, g_report.heavyTiming);
        if (step == 0 || step == 100 || step == 300 || step == HEAVY_STEPS - 1)
        {
            g_report.heavyPairCounts[sampleIndex] = SamplePairCounts(step, world);
            ++sampleIndex;
        }
    }
    g_report.heavyChecksum = world.GenerateChecksum();
    Log("255 bodies, %d steps: %08" PRIx32 "\n", HEAVY_STEPS, g_report.heavyChecksum);
    LogTiming("255", g_report.heavyTiming);
}

/**
 * @brief Same scene, run to a resting pile, then profiled with StepPhase ticks: slower than RunHeavyPile because
 * of the extra cpuGetTiming() reads, so its own Step timing isn't part of the reported avg/max ms.
 */
void RunHeavyPileProfile()
{
    neko::PhysicsWorld world{};
    BuildHeavyPile(world);

    constexpr int warmupSteps = HEAVY_STEPS - 120;
    Timing warmup;
    for (int step = 0; step < warmupSteps; ++step)
    {
        TimedStep(world, warmup);
    }

    neko::g_stepPhaseTicks = &g_report.heavyPhaseTicks;
    Timing profiled;
    g_report.heavyPhaseSteps = HEAVY_STEPS - warmupSteps;
    for (int step = 0; step < g_report.heavyPhaseSteps; ++step)
    {
        TimedStep(world, profiled);
    }
    neko::g_stepPhaseTicks = nullptr;

    const auto average = static_cast<std::uint32_t>(profiled.total / static_cast<std::uint64_t>(profiled.count));
    Log("255 phase breakdown, resting pile, %d steps (avg %" PRIu32 "us/step with profiling on):\n",
        g_report.heavyPhaseSteps, timerTicks2usec(average));
    constexpr const char* names[] = { "integration", "broadphase", "detectContact", "resolve", "sortAndNotify" };
    for (std::size_t i = 0; i < static_cast<std::size_t>(neko::StepPhase::Count); ++i)
    {
        const auto ticksPerStep =
            g_report.heavyPhaseTicks.ticks[i] / static_cast<std::uint32_t>(g_report.heavyPhaseSteps);
        Log("  %-14s %6" PRIu32 "us/step\n", names[i], timerTicks2usec(ticksPerStep));
    }
}

} // namespace

// Reached exactly once, after every result has been logged: the breakpoint a host-side gdb session sets before
// letting the ARM9 run (melonDS's GDB stub, see melonDS.toml [Instance0.Gdb]), since results live in g_report.
[[gnu::noinline]] void TestsDone()
{
    asm volatile("" ::: "memory");
}

int main()
{
    consoleDemoInit();
    Log("NekoPhysics DS test\n");
    g_report.hooksOk = CheckMathHooks();
    g_report.pileOk = RunFallingPile();
    RunHeavyPile();
    RunHeavyPileProfile();
    Log("DONE %s\n", g_report.hooksOk && g_report.pileOk ? "OK" : "FAIL");
    TestsDone();

    while (pmMainLoop())
    {
        swiWaitForVBlank();
    }
    return 0;
}
