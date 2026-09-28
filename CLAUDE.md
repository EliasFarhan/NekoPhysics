# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

NekoPhysics is a small deterministic 2D physics library in C++20 with a quadtree broadphase and circle/AABB colliders. The end goal is to run it on the Nintendo DS (devkitPro/devkitARM, ARM946E-S, no FPU) with rollback netcode. `neko::Scalar` is chosen by the CMake cache variable `NEKO_SCALAR`: `FIXED16` (default, 16.16 fixed point `neko::Fixed16`, the DS type), `FLOAT` (plain `float`, PC convenience) or `SIXIT` (`sixit::dmath::ieee_float_inline_asm`, x86 only, `externals/sixit`). Tests must pass with both `FIXED16` and `FLOAT`. Layout:
- `main/samples/`: `NekoPhysicsSamples`, the six samples as a platform-neutral static library (no SDL, ImGui or fmt). A sample simulates only in `FixedUpdate` and describes itself in a world-space `DrawList` (`draw_list.h`, palette colors). Scenes come from a fixed-seed `SampleRng` and integer `Ratio`s (no float, deterministic), and bounds and body counts come from `sample_config.h` (per platform, `NEKO_PLATFORM_NDS`). Built on PC and for the DS.
- `main/sdl/`: the PC host (`Sample` exe): SDL3 window, ImGui menu, fixed-step accumulator, `SdlSampleDrawer`.
- `nds/`: the DS test ROM (`NekoPhysicsDsTest`, golden checksums and timings) and `nds/samples/`, the DS samples ROM (`NekoPhysicsDsSamples`: `NdsSampleDrawer` with hardware sprites plus a BG3 8bpp bitmap, and a menu and stats console on the bottom screen). Nothing on the DS samples path may link soft-float: check the ELF for `__aeabi_f*`/`__aeabi_d*` and print with `iprintf`/`sniprintf`.
- `tests/`: GoogleTest executables, one per file, each with its own `main`. `samples_test` also links `NekoPhysicsSamples` (sample determinism).

## Reviews, discussions and plans live in the Obsidian vault

**Folder:** `H:/vault/sae/Projets/NekoPhysics/`. Write new reviews, design discussions and plans there (with YAML frontmatter like the existing notes) and update their status there. Do not add review or plan documents to this repo.

- `NekoPhysics review.md`: code review from 2026-09-18 (verified on `fd65db2`), findings S1–S5 with block ids (`^s1-1` …) and effort sizes.
- `NekoPhysics review tasks.md`: sortable findings table. Mark items 🟢 there when fixed (with the commit hash).
- `NekoPhysics DS review.md`: Nintendo DS review from 2026-09-27 (verified on `49fbd8f`), findings D1–D5.
- `Plans/`: implementation plans. `Plans/NekoPhysics DS port plan.md` is the current roadmap (phases 0–6); update its status table as steps land.

## Build

vcpkg manifest; optional features are toggled by CMake options: `USE_SAMPLES`, `USE_TESTING`, `ENABLE_PROFILING` (Tracy), plus `NEKO_SCALAR` (see above). There are no submodules: `externals/sixit` is vendored.

```
cmake --preset debug          # FIXED16; needs VCPKG_ROOT
cmake --build --preset debug
ctest --preset debug
cmake --preset debug-float    # same with NEKO_SCALAR=FLOAT
cmake --preset nds            # DS ROMs (build/nds/nds/NekoPhysicsDsTest.nds, build/nds/nds/samples/NekoPhysicsDsSamples.nds); needs DEVKITPRO, no vcpkg
cmake --build --preset nds
```

As of 2026-09-18 `NEKO_SCALAR=SIXIT` fails to build with MSVC 14.51 and clang (sixit's bundled fmt, non-constexpr `Scalar`); see finding S1.8 in the vault review.


## Warnings

`cmake/CompilerWarnings.cmake` defines `neko_set_project_warnings(<target>)`, applied to the library, the samples and every test. New targets must call it. `NEKO_WARNINGS_AS_ERRORS` is ON (also forced by the presets): MSVC `/W4` + extras are clean for the library, tests and samples; the clang list is clean for the library and tests (checked with emsdk clang against the MSVC headers). The GCC-only flags have not been run yet. Don't silence warnings with casts at call sites when the fix belongs in the types (indices are `neko::Index` = `std::uint16_t`).
