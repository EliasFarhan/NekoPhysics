# NekoPhysics

A small deterministic 2D physics library in C++ with a quadtree broadphase and
circle/AABB colliders, meant to run on the Nintendo DS with rollback netcode.

## Build

The PC build uses vcpkg (`VCPKG_ROOT` must be set):

```
cmake --preset debug
cmake --build --preset debug
ctest --preset debug
```

See `CMakePresets.json` for the other presets (`debug-float`, `release`,
`nds`...).

## Documentation

The API reference of the library (`include/neko`) and of the samples
(`main/samples`) is generated with [MrDocs](https://mrdocs.com) from the `///`
comments in the headers.

1. Install MrDocs: download the archive for your OS from the
   [releases](https://github.com/cppalliance/mrdocs/releases), extract it and
   put its `bin` folder on `PATH` (or set `MRDOCS_ROOT` to the extracted
   folder).
2. Configure with `NEKO_BUILD_DOCS=ON` and build the `docs` target:

   ```
   cmake --preset debug -DNEKO_BUILD_DOCS=ON
   cmake --build --preset debug --target docs
   ```

3. Open `build/debug/docs/index.html` in a browser. Start with the `neko`
   namespace page, which gives an overview of the library.

## Tutorial browser

Build and launch all 17 chapter demos and 68 exercise solutions in one desktop
executable:

```sh
cmake --preset debug
cmake --build --preset debug --target Tutorials
./build/debug/tutorials/Tutorials
```

Use the Samples menu to expand a chapter and select an experiment. Search filters
by experiment name or chapter; the existing sliders, readouts and Restart button
work for every entry. Switching restarts the simulation and retains edited
parameters for the duration of the application.

```sh
./build/debug/tutorials/Tutorials --list
./build/debug/tutorials/Tutorials --sample 00_bounce/explicit_euler
./build/debug/tutorials/Tutorials --sample 12_aiming/demo
./build/debug/tutorials/Tutorials --help
```

Listing, help and invalid arguments do not open a window. IDs follow
`<chapter>/<exercise>`; chapter demos use `<chapter>/demo`.

Individual `course_*` launchers are available by configuring with
`-DNEKO_BUILD_INDIVIDUAL_TUTORIALS=ON` (off by default). The tutorial browser
requires `USE_SAMPLES=ON`; the platform-neutral tutorial library and tests still
build with `USE_SAMPLES=OFF` and `USE_TESTING=ON`.

When adding a tutorial, register its ID, chapter, name and factory in
`tutorials/tutorial_catalog.cc`. The catalog coverage test checks the registration
against the chapter and solution launchers.
