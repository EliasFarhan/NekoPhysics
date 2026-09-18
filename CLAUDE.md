# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

NekoPhysics is a small deterministic 2D physics library in C++20 with a quadtree broadphase and circle/AABB colliders. `neko::Scalar` is `sixit::dmath::ieee_float_inline_asm` (deterministic float, `externals/sixit`) when `USE_SIXIT=ON` (default), else plain `float`. The older 16.16 fixed-point `neko::Fixed16` (with lookup tables for `Sin`/`Cos`/`Sqrt`) is still in the tree but only used by `tests/fixed_test.cpp`. `main/` holds SDL3/ImGui samples; `tests/` holds GoogleTest executables (one per file).

## Reviews, discussions and plans live in the Obsidian vault

**Folder:** `H:/vault/sae/Projets/NekoPhysics/`. Write new reviews, design discussions and plans there (with YAML frontmatter like the existing notes) and update their status there. Do not add review or plan documents to this repo.

- `NekoPhysics review.md`: code review from 2026-09-18 (verified on `fd65db2`), findings S1–S5 with block ids (`^s1-1` …) and effort sizes.
- `NekoPhysics review tasks.md`: sortable findings table. Mark items 🟢 there when fixed (with the commit hash).
- `Plans/`: implementation plans.

## Build

vcpkg manifest; optional features are toggled by CMake options: `USE_SAMPLES`, `USE_TESTING`, `ENABLE_PROFILING` (Tracy), `USE_SIXIT`. Submodules are required: `git submodule update --init --recursive`.

```
cmake -S . -B build -G Ninja -DCMAKE_TOOLCHAIN_FILE=<vcpkg>/scripts/buildsystems/vcpkg.cmake -DUSE_TESTING=ON -DUSE_SIXIT=OFF
cmake --build build
ctest --test-dir build
```

As of 2026-09-18 the default `USE_SIXIT=ON` build fails with MSVC 14.51 and clang (sixit's bundled fmt, non-constexpr `Scalar`); see finding S1.8 in the vault review. Use `USE_SIXIT=OFF` until fixed.

Configure runs `scripts/gen_fixed_table.py` (Python 3 required), which rewrites `src/generated/`.

## Warnings

`cmake/CompilerWarnings.cmake` defines `neko_set_project_warnings(<target>)`, applied to the library, the samples and every test. New targets must call it. `NEKO_WARNINGS_AS_ERRORS` is OFF until the existing signed/unsigned conversion warnings (C4365 / `-Wsign-conversion`, from `std::int32_t` indices used as vector subscripts) are fixed; the goal is to turn it ON. Don't silence warnings with casts at call sites when the fix belongs in the index types.
