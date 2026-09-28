// DS samples ROM: the six NekoPhysicsSamples on the top screen (NdsSampleDrawer), a menu and timings on the bottom
// screen console. One fixed step per VBlank (60 Hz = Sample::fixedDeltaTime), no accumulator.
//
// Everything printed goes through newlib's integer-only iprintf/sniprintf, so the float printf code (and the
// soft-float routines it pulls in) stays out of the ROM.
#include "nds_sample_drawer.h"

#include "samples/sample.h"
#include "samples/sample_config.h"

#include <nds.h>

#include <array>
#include <cinttypes>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace
{

constexpr int STATS_WINDOW = 60;
/**
 * @brief The frame period is measured with calico's system tick counter (tickGetCount, TICK_FREQ = bus clock / 64).
 * Hardware timers 2 and 3 run that counter (cpuStartTiming reads it too), so they must not be reprogrammed.
 */
constexpr std::uint64_t TICK_HZ_X10 = std::uint64_t{ TICK_FREQ } * 10;
/**
 * @brief VBlanks per second (59.8261 Hz) x 10
 */
constexpr std::uint32_t VBLANK_HZ_X10 = 598;

constexpr int CONSOLE_COLUMNS = 32;
constexpr int MENU_ROW = 2;
constexpr int STATS_ROW = 10;
constexpr int HELP_ROW = 20;

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
 * @brief Values shown on the console, refreshed every STATS_WINDOW frames
 */
struct DisplayedStats
{
    std::uint32_t stepAverageUs = 0;
    std::uint32_t stepMaxUs = 0;
    std::uint32_t drawAverageUs = 0;
    std::uint32_t drawMaxUs = 0;
    std::uint32_t fpsX10 = 0;
};

/**
 * @brief Console rows as last printed: a row is only reprinted when its text changes, to keep console time down
 */
class ConsoleRows
{
public:
    __attribute__((format(printf, 3, 4))) void Print(int row, const char* format, ...)
    {
        std::array<char, CONSOLE_COLUMNS + 1> line{};
        va_list args;
        va_start(args, format);
        vsniprintf(line.data(), line.size(), format, args);
        va_end(args);
        // Pad to the full width, so a shorter text erases the previous one
        const std::size_t length = std::strlen(line.data());
        std::memset(line.data() + length, ' ', CONSOLE_COLUMNS - length);
        line[CONSOLE_COLUMNS] = '\0';
        auto& previous = rows_[static_cast<std::size_t>(row)];
        if (std::strcmp(previous.data(), line.data()) != 0)
        {
            previous = line;
            // The last column is left empty: printing into it would wrap the cursor to the next row
            iprintf("\x1b[%d;0H%.31s", row, line.data());
        }
    }

private:
    std::array<std::array<char, CONSOLE_COLUMNS + 1>, 24> rows_{};
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
    const neko::SampleContext context{ neko::sample_config::WORLD_BOUNDS };
    neko::DrawList drawList;
    drawList.Reserve(neko::sample_config::MAX_DRAW_CIRCLES, neko::sample_config::MAX_DRAW_BOXES,
        neko::sample_config::MAX_DRAW_LINES);
    std::size_t cursor = 0;
    registry.Select(cursor, context);

    ConsoleRows rows;
    rows.Print(0, "NekoPhysics DS samples");
    rows.Print(HELP_ROW, "Up/Down + A: select");
    rows.Print(HELP_ROW + 1, "L/R: cycle  START: restart");

    TimingWindow stepTiming;
    TimingWindow drawTiming;
    std::uint64_t windowFrameTicks = 0;
    DisplayedStats displayed;
    std::uint64_t lastFrameTick = tickGetCount();

    while (pmMainLoop())
    {
        scanKeys();
        const auto keys = keysDown();
        const std::size_t sampleCount = registry.size();
        if ((keys & KEY_UP) != 0)
        {
            cursor = (cursor + sampleCount - 1) % sampleCount;
        }
        if ((keys & KEY_DOWN) != 0)
        {
            cursor = (cursor + 1) % sampleCount;
        }
        if ((keys & (KEY_L | KEY_R)) != 0)
        {
            const std::size_t current = registry.currentIndex();
            cursor = (keys & KEY_L) != 0 ? (current + sampleCount - 1) % sampleCount : (current + 1) % sampleCount;
        }
        if ((keys & (KEY_A | KEY_L | KEY_R)) != 0 && cursor != registry.currentIndex())
        {
            registry.Select(cursor, context);
            stepTiming = {};
            drawTiming = {};
            windowFrameTicks = 0;
        }
        if ((keys & KEY_START) != 0)
        {
            registry.Select(registry.currentIndex(), context);
        }

        cpuStartTiming(0);
        registry.FixedUpdate();
        stepTiming.Add(cpuEndTiming());

        cpuStartTiming(0);
        registry.Draw(drawList);
        drawer.Render(drawList);
        drawTiming.Add(cpuEndTiming());

        for (std::size_t i = 0; i < sampleCount; ++i)
        {
            const char marker = i == registry.currentIndex() ? '*' : ' ';
            rows.Print(MENU_ROW + static_cast<int>(i), "%c%c %s", i == cursor ? '>' : ' ', marker, registry.name(i));
        }
        const auto& stats = drawer.stats();
        rows.Print(STATS_ROW, "Shapes  %3zu circles %2zu boxes", drawList.circles.size(), drawList.boxes.size());
        rows.Print(STATS_ROW + 1, "Lines   %3zu", drawList.lines.size());
        rows.Print(STATS_ROW + 2, "Sprites %3d used %3d dropped", stats.spritesUsed, stats.droppedSprites);
        rows.Print(STATS_ROW + 3, "Bitmap  %3d shapes", stats.bitmapShapes);
        rows.Print(STATS_ROW + 4, "Redraws %" PRIu32, stats.bitmapRedraws);

        swiWaitForVBlank();
        drawer.Present();

        const std::uint64_t frameTick = tickGetCount();
        windowFrameTicks += frameTick - lastFrameTick;
        lastFrameTick = frameTick;
        if (stepTiming.count == STATS_WINDOW)
        {
            displayed.stepAverageUs = stepTiming.AverageUs();
            displayed.stepMaxUs = stepTiming.MaxUs();
            displayed.drawAverageUs = drawTiming.AverageUs();
            displayed.drawMaxUs = drawTiming.MaxUs();
            displayed.fpsX10 = windowFrameTicks == 0
                ? 0
                : static_cast<std::uint32_t>(TICK_HZ_X10 * STATS_WINDOW / windowFrameTicks);
            stepTiming = {};
            drawTiming = {};
            windowFrameTicks = 0;
        }
        rows.Print(STATS_ROW + 6, "Step %5" PRIu32 " us avg %5" PRIu32 " max", displayed.stepAverageUs,
            displayed.stepMaxUs);
        rows.Print(STATS_ROW + 7, "Draw %5" PRIu32 " us avg %5" PRIu32 " max", displayed.drawAverageUs,
            displayed.drawMaxUs);
        // VBlanks per frame x 10, rounded
        const std::uint32_t vblanksPerFrameX10 =
            displayed.fpsX10 == 0 ? 0 : (VBLANK_HZ_X10 * 10 + displayed.fpsX10 / 2) / displayed.fpsX10;
        rows.Print(STATS_ROW + 8, "FPS  %2" PRIu32 ".%" PRIu32 " (%" PRIu32 ".%" PRIu32 " VBlank/frame)",
            displayed.fpsX10 / 10, displayed.fpsX10 % 10, vblanksPerFrameX10 / 10, vblanksPerFrameX10 % 10);
    }
    return 0;
}
