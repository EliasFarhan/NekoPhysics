// DS test ROM: DS == PC determinism of the falling pile scene, self-check of the hardware math hooks, Step timings.
// Results go to the bottom screen and to the no$gba debug output (nocashMessage).
#include "physics/physics.h"
#include "scenes/falling_pile.h"

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

// Same values as tests/scene_test.cpp
constexpr std::pair<int, std::uint32_t> GOLDEN_CHECKSUMS[] = {
    { 0, 0x61c2e799u },
    { 1, 0xfa00ce4cu },
    { 60, 0xbee5c5bau },
    { 120, 0x2cc11c14u },
    { 300, 0xe2ff67d3u },
    { 600, 0xa68114abu },
};

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
    Timing timing;
    bool ok = true;
    int step = 0;
    for (const auto& [expectedStep, expected] : GOLDEN_CHECKSUMS)
    {
        while (step < expectedStep)
        {
            TimedStep(world, timing);
            ++step;
        }
        const std::uint32_t checksum = world.GenerateChecksum();
        ok = ok && checksum == expected;
        Log("%3d %08" PRIx32 " %08" PRIx32 " %s\n", step, checksum, expected, checksum == expected ? "OK" : "FAIL");
    }
    Log("pile: %s\n", ok ? "OK" : "FAIL");
    LogTiming("67", timing);
    return ok;
}

/**
 * @brief Timing only: 3 static boxes and 252 dynamic bodies, near the default capacity of 256 bodies
 */
void RunHeavyPile()
{
    using neko::Scalar;
    using neko::scenes::Ratio;
    neko::PhysicsWorld world{};
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

    Timing timing;
    for (int step = 0; step < 600; ++step)
    {
        TimedStep(world, timing);
    }
    Log("255 bodies, 600 steps: %08" PRIx32 "\n", world.GenerateChecksum());
    LogTiming("255", timing);
}

} // namespace

int main()
{
    consoleDemoInit();
    Log("NekoPhysics DS test\n");
    const bool hooksOk = CheckMathHooks();
    const bool pileOk = RunFallingPile();
    RunHeavyPile();
    Log("DONE %s\n", hooksOk && pileOk ? "OK" : "FAIL");

    while (pmMainLoop())
    {
        swiWaitForVBlank();
    }
    return 0;
}
