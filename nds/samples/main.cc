// DS samples ROM: the six NekoPhysicsSamples on the top screen (NdsSampleDrawer), a menu and timings on the bottom
// screen console. One fixed step per VBlank (60 Hz = Sample::kFixedDeltaTime), no accumulator.
//
// Everything printed goes through newlib's integer-only iprintf/sniprintf, so the float printf code (and the
// soft-float routines it pulls in) stays out of the ROM.
#include "nds/samples/nds_sample_drawer.h"

#include "main/samples/sample.h"
#include "main/samples/sample_config.h"

#include <nds.h>

#include <array>
#include <cinttypes>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace
{

constexpr int kStatsWindow = 60;
/**
 * @brief The frame period is measured with calico's system tick counter (tickGetCount, TICK_FREQ = bus clock / 64).
 * Hardware timers 2 and 3 run that counter (cpuStartTiming reads it too), so they must not be reprogrammed.
 */
constexpr std::uint64_t kTickHzX10 = std::uint64_t{ TICK_FREQ } * 10;
/**
 * @brief VBlanks per second (59.8261 Hz) x 10
 */
constexpr std::uint32_t kVblankHzX10 = 598;

constexpr int kConsoleColumns = 32;
constexpr int kMenuRow = 2;
constexpr int kStatsRow = 10;
constexpr int kHelpRow = 20;

struct TimingWindow
{
    std::uint32_t total = 0;
    std::uint32_t max = 0;
    int count = 0;

    void Add(std::uint32_t ticks)
    {
        total += ticks;
        max = ticks > max ? ticks : max;
        ++count;
    }
    [[nodiscard]] std::uint32_t AverageUs() const
    {
        return count == 0 ? 0 : timerTicks2usec(total / static_cast<std::uint32_t>(count));
    }
    [[nodiscard]] std::uint32_t MaxUs() const { return timerTicks2usec(max); }
};

/**
 * @brief Values shown on the console, refreshed every kStatsWindow frames
 */
struct DisplayedStats
{
    std::uint32_t step_average_us = 0;
    std::uint32_t step_max_us = 0;
    std::uint32_t draw_average_us = 0;
    std::uint32_t draw_max_us = 0;
    std::uint32_t fps_x10 = 0;
};

/**
 * @brief Console rows as last printed: a row is only reprinted when its text changes, to keep console time down
 */
class ConsoleRows
{
public:
    __attribute__((format(printf, 3, 4))) void Print(int row, const char* format, ...)
    {
        std::array<char, kConsoleColumns + 1> line{};
        va_list args;
        va_start(args, format);
        vsniprintf(line.data(), line.size(), format, args);
        va_end(args);
        // Pad to the full width, so a shorter text erases the previous one
        const std::size_t length = std::strlen(line.data());
        std::memset(line.data() + length, ' ', kConsoleColumns - length);
        line[kConsoleColumns] = '\0';
        auto& previous = rows_[static_cast<std::size_t>(row)];
        if (std::strcmp(previous.data(), line.data()) != 0)
        {
            previous = line;
            // The last column is left empty: printing into it would wrap the cursor to the next row
            iprintf("\x1b[%d;0H%.31s", row, line.data());
        }
    }

private:
    std::array<std::array<char, kConsoleColumns + 1>, 24> rows_{};
};

} // namespace

int main()
{
    neko::NdsSampleDrawer drawer;
    drawer.Init();

    videoSetModeSub(MODE_0_2D);
    vramSetBankC(VRAM_C_SUB_BG);
    PrintConsole console;
    consoleInit(&console, 0, BgType_Text4bpp, BgSize_T_256x256, 31, 0, false, true);
    consoleSelect(&console);

    neko::SampleRegistry registry;
    const neko::SampleContext context{ neko::sample_config::kWorldBounds };
    neko::DrawList draw_list;
    draw_list.Reserve(neko::sample_config::kMaxDrawCircles, neko::sample_config::kMaxDrawBoxes,
        neko::sample_config::kMaxDrawLines);
    std::size_t cursor = 0;
    registry.Select(cursor, context);

    ConsoleRows rows;
    rows.Print(0, "NekoPhysics DS samples");
    rows.Print(kHelpRow, "Up/Down + A: select");
    rows.Print(kHelpRow + 1, "L/R: cycle  START: restart");

    TimingWindow step_timing;
    TimingWindow draw_timing;
    std::uint64_t window_frame_ticks = 0;
    DisplayedStats displayed;
    std::uint64_t last_frame_tick = tickGetCount();

    while (pmMainLoop())
    {
        scanKeys();
        const auto keys = keysDown();
        const std::size_t sample_count = registry.size();
        if ((keys & KEY_UP) != 0)
        {
            cursor = (cursor + sample_count - 1) % sample_count;
        }
        if ((keys & KEY_DOWN) != 0)
        {
            cursor = (cursor + 1) % sample_count;
        }
        if ((keys & (KEY_L | KEY_R)) != 0)
        {
            const std::size_t current = registry.current_index();
            cursor = (keys & KEY_L) != 0 ? (current + sample_count - 1) % sample_count : (current + 1) % sample_count;
        }
        if ((keys & (KEY_A | KEY_L | KEY_R)) != 0 && cursor != registry.current_index())
        {
            registry.Select(cursor, context);
            step_timing = {};
            draw_timing = {};
            window_frame_ticks = 0;
        }
        if ((keys & KEY_START) != 0)
        {
            registry.Select(registry.current_index(), context);
        }

        cpuStartTiming(0);
        registry.FixedUpdate();
        step_timing.Add(cpuEndTiming());

        cpuStartTiming(0);
        registry.Draw(draw_list);
        drawer.Render(draw_list);
        draw_timing.Add(cpuEndTiming());

        for (std::size_t i = 0; i < sample_count; ++i)
        {
            const char marker = i == registry.current_index() ? '*' : ' ';
            rows.Print(kMenuRow + static_cast<int>(i), "%c%c %s", i == cursor ? '>' : ' ', marker, registry.name(i));
        }
        const auto& stats = drawer.stats();
        rows.Print(kStatsRow, "Shapes  %3zu circles %2zu boxes", draw_list.circles.size(), draw_list.boxes.size());
        rows.Print(kStatsRow + 1, "Lines   %3zu", draw_list.lines.size());
        rows.Print(kStatsRow + 2, "Sprites %3d used %3d dropped", stats.sprites_used, stats.dropped_sprites);
        rows.Print(kStatsRow + 3, "Bitmap  %3d shapes", stats.bitmap_shapes);
        rows.Print(kStatsRow + 4, "Redraws %" PRIu32, stats.bitmap_redraws);

        swiWaitForVBlank();
        drawer.Present();

        const std::uint64_t frame_tick = tickGetCount();
        window_frame_ticks += frame_tick - last_frame_tick;
        last_frame_tick = frame_tick;
        if (step_timing.count == kStatsWindow)
        {
            displayed.step_average_us = step_timing.AverageUs();
            displayed.step_max_us = step_timing.MaxUs();
            displayed.draw_average_us = draw_timing.AverageUs();
            displayed.draw_max_us = draw_timing.MaxUs();
            displayed.fps_x10 = window_frame_ticks == 0
                ? 0
                : static_cast<std::uint32_t>(kTickHzX10 * kStatsWindow / window_frame_ticks);
            step_timing = {};
            draw_timing = {};
            window_frame_ticks = 0;
        }
        rows.Print(kStatsRow + 6, "Step %5" PRIu32 " us avg %5" PRIu32 " max", displayed.step_average_us,
            displayed.step_max_us);
        rows.Print(kStatsRow + 7, "Draw %5" PRIu32 " us avg %5" PRIu32 " max", displayed.draw_average_us,
            displayed.draw_max_us);
        // VBlanks per frame x 10, rounded
        const std::uint32_t vblanks_per_frame_x10 =
            displayed.fps_x10 == 0 ? 0 : (kVblankHzX10 * 10 + displayed.fps_x10 / 2) / displayed.fps_x10;
        rows.Print(kStatsRow + 8, "FPS  %2" PRIu32 ".%" PRIu32 " (%" PRIu32 ".%" PRIu32 " VBlank/frame)",
            displayed.fps_x10 / 10, displayed.fps_x10 % 10, vblanks_per_frame_x10 / 10, vblanks_per_frame_x10 % 10);
    }
    return 0;
}
