# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

NekoPhysics is a small deterministic 2D physics library in C++20 with a quadtree broadphase and circle/AABB colliders. The end goal is to run it on the Nintendo DS (devkitPro/devkitARM, ARM946E-S, no FPU) with rollback netcode. `neko::Scalar` is chosen by the CMake cache variable `NEKO_SCALAR`: `FIXED16` (default, 16.16 fixed point `neko::Fixed16`, the DS type), `FLOAT` (plain `float`, PC convenience) or `SIXIT` (`sixit::dmath::ieee_float_inline_asm`, x86 only, `externals/sixit`). Tests must pass with both `FIXED16` and `FLOAT`. Layout:
- `main/samples/`: `NekoPhysicsSamples`, the six samples as a platform-neutral static library (no SDL, ImGui or fmt). A sample simulates only in `FixedUpdate` and describes itself in a world-space `DrawList` (`draw_list.h`, palette colors). Scenes come from a fixed-seed `SampleRng` and integer `Ratio`s (no float, deterministic), and bounds and body counts come from `sample_config.h` (per platform, `NEKO_PLATFORM_NDS`). Built on PC and for the DS. Everything is y-up, like the physics (the drawers flip y). A sample can expose `SampleParameter`s (integer `value / scale`, changed between runs only: the registry restarts the sample) and `SampleReadout`s (measured values); both hosts edit and print them with integer-only `FormatDecimal`.
- `main/sdl/`: the PC host library `NekoPhysicsSdlHost` (`RunSdlHost`: SDL3 window, ImGui menu with parameter sliders and readouts, fixed-step accumulator, `SdlSampleDrawer`) and the `Sample` exe.
- `tutorials/NN_<slug>/`: one chapter of the NekoPhysics articles (written in `GPR_SAE_Slides/nekophysics/`, not here): a platform-neutral `Sample` in `NekoPhysicsTutorials` (namespace `neko::tutorial`, `//` comments, not in MrDocs) and `course_NN_<slug>` running it on the SDL host (`neko_add_tutorial`). Tested by `tests/tutorials_test.cc`.
- `nds/`: the DS test ROM (`NekoPhysicsDsTest`, golden checksums and timings) and `nds/samples/`, the DS samples ROM (`NekoPhysicsDsSamples`: `NdsSampleDrawer` with hardware sprites plus a BG3 8bpp bitmap, and a menu and stats console on the bottom screen). Nothing on the DS samples path may link soft-float: check the ELF for `__aeabi_f*`/`__aeabi_d*` and print with `iprintf`/`sniprintf`.
- `tests/`: GoogleTest executables, one per file, each with its own `main`. `samples_test` also links `NekoPhysicsSamples` (sample determinism and FIXED16 golden checksums), `tutorials_test` links `NekoPhysicsTutorials`. FIXED16 goldens (falling pile world and trajectory checksums in `tests/scenes/falling_pile.h`, samples in `samples_test.cc`) are re-pinned only on purpose, saying why in the commit. `conventions_test` checks the conventions documented in `physics.h` (units, y-up, normal direction, touching, no slop, Fixed16 ranges). New test files need a reconfigure (the test glob runs at configure time).

## Reviews, discussions and plans live in the Obsidian vault

**Folder:** `H:/vault/sae/Projets/NekoPhysics/` on Windows, `~/Documents/Vault/SAE/Projets/NekoPhysics/` on Linux. Write new reviews, design discussions and plans there (with YAML frontmatter like the existing notes) and update their status there. Do not add review or plan documents to this repo.

- `NekoPhysics review.md`: code review from 2026-09-18 (verified on `fd65db2`), findings S1–S5 with block ids (`^s1-1` …) and effort sizes.
- `NekoPhysics review tasks.md`: sortable findings table. Mark items 🟢 there when fixed (with the commit hash).
- `NekoPhysics DS review.md`: Nintendo DS review from 2026-09-27 (verified on `49fbd8f`), findings D1–D5.
- `NekoPhysics feature and teaching roadmap.md`: the feature roadmap (milestones M0–M8: rotation, contacts and friction, joints, gameplay, CCD) and the article series, with the implementation decisions of 2026-10-16. `NekoPhysics M0 baseline.md`: measured baseline (tests, goldens, DS sizes) to compare later milestones against.
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
cmake --preset debug -DNEKO_BUILD_DOCS=ON   # API reference with MrDocs (mrdocs on PATH or MRDOCS_ROOT)
cmake --build --preset debug --target docs  # -> build/debug/docs/index.html, config in docs/mrdocs.yml
```

The `docs` target warns on undocumented symbols, doc errors and broken references: keep it warning-free when touching public headers.

After building the docs, `python3 docs/check_links.py build/debug/docs` checks the generated pages: dead links, link text that disagrees with its target, prose links landing on a constructor page and code spans ending with punctuation are errors (exit code 1). It also lists the documented type names left as plain text, to review by hand (ordinary English, or a missing `@ref`).

As of 2026-09-18 `NEKO_SCALAR=SIXIT` fails to build with MSVC 14.51 and clang (sixit's bundled fmt, non-constexpr `Scalar`); see finding S1.8 in the vault review.


## Style

The code follows the [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html), migrated on branch `google-style` (plan in the vault: `Plans/NekoPhysics Google style plan.md`). Project choices:
- `.clang-format` (Google, 80 columns) formats everything outside `externals/`. Run `clang-format -i` on touched files before committing.
- `.clang-tidy` checks `google-*` and Google naming (naming warnings are errors): `snake_case` variables and struct fields, `snake_case_` private members, `kCamelCase` constants and enumerators, `CamelCase` functions, `g_` for mutable globals. Accessors and mutators are `snake_case` (`x()`, `set_x()`, `body()`, `touching_pairs()`); clang-tidy accepts any snake_case method, so keep snake_case for accessors only. Run it with `run-clang-tidy -p build/debug`; on a GCC compile DB (Linux) add `-extra-arg=-Wno-unknown-warning-option`, since the GCC-only warning flags (`-Wduplicated-*`, `-Wlogical-op`, `-Wuseless-cast`) are unknown to clang. For the DS code, export a compile DB (`cmake --preset nds -B build/nds-tidy -DCMAKE_EXPORT_COMPILE_COMMANDS=ON`) and pass `--target=arm-none-eabi` plus devkitARM's include paths as `-isystem`. Never `-fix` a TU that has compile errors: clang-tidy also applies compiler fix-its.
- Sources are `.cc`. Headers use `#define` guards named from the path (`NEKO_MATH_VEC2_H_`, `MAIN_SAMPLES_SAMPLE_H_`). Includes name the full path: `"neko/..."` for the library (`include/neko/`), and the repo-root path for the rest (`"main/samples/sample.h"`, `"tests/scenes/falling_pile.h"`).
- Comments are `//`, except the public API in `include/neko` and `main/samples` headers: every symbol MrDocs extracts (namespace `neko`, outside `neko::internal`) gets a `///` doc comment, written for game programming students (concepts, units, determinism caveats, not just the signature). First sentence is the brief; use `@param`, `@return`, `@note`, `@see`, `@ref` and `@code`/`@endcode` (Markdown fences don't render). Implementation comments stay `//`.
- Links in doc comments (MrDocs 0.8): a documented symbol named in prose is `@ref Type` or `@ref Type::member`, not backticks, and `@see` needs it too (`@see @ref Type`). Don't write `@ref Self` inside the comment of class `Self` or of its members: it resolves to the constructor, so keep backticks there. Types in a synopsis that MrDocs can't link (initializers, template arguments of `std` types, `auto` constants) are named with `@ref` in the comment. Functions returning a documented type need an explicit `@return`, or MrDocs shows the brief of that type. `@p name` takes the punctuation that follows into its code span and `[a, b)` gains a stray `]`: use backticks when punctuation follows the name and write intervals in words.
- Value types with behavior are classes with private fields and accessors: `Vec2` (`x()`, `set_x()`), `Aabb`, `Circle`, the index types (`index()`, explicit constructor from `Index`) and `ColliderPair`. Passive data (`Body`, `Collider`, configs...) stays structs with public `snake_case` fields. `Fixed`, `Vec2`, `Radian` and `Degree` assignments are lvalue-only, so `v.x() += d` doesn't compile; use the setters.
- No exceptions or RTTI in the library (`-fno-exceptions -fno-rtti`); allocation failure aborts. Single-argument constructors are `explicit`.
- Standard-interface names are kept with `NOLINT` (`std::numeric_limits<Fixed>`, `StandardAllocator::allocate`, `is_fixed_v`).

## Warnings

`cmake/CompilerWarnings.cmake` defines `neko_set_project_warnings(<target>)`, applied to the library, the samples and every test. New targets must call it. `NEKO_WARNINGS_AS_ERRORS` is ON (also forced by the presets): MSVC `/W4` + extras are clean for the library, tests and samples; the clang list is clean for the library and tests (checked with emsdk clang against the MSVC headers). GCC 16.2 (Linux, with the GCC-only flags) and devkitARM are clean too (2026-10-04). Don't silence warnings with casts at call sites when the fix belongs in the types (indices are `neko::Index` = `std::uint16_t`).
