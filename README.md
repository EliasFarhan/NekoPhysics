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
