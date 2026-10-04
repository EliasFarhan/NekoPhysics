# DS test ROM: golden checksums of the falling pile scene, hardware math hooks self-check and Step timings.
# Only built with devkitPro's NDS toolchain (-DCMAKE_TOOLCHAIN_FILE=$DEVKITPRO/cmake/NDS.cmake).
add_executable(NekoPhysicsDsTest main.cc)
target_link_libraries(NekoPhysicsDsTest PRIVATE NekoPhysics)
target_include_directories(NekoPhysicsDsTest PRIVATE ${PROJECT_SOURCE_DIR})
target_compile_options(NekoPhysicsDsTest PRIVATE -marm -fno-exceptions -fno-rtti)
neko_set_project_warnings(NekoPhysicsDsTest)
nds_create_rom(NekoPhysicsDsTest NAME "NekoPhysics" SUBTITLE1 "DS test ROM" SUBTITLE2 "checksums and timings")
add_subdirectory(samples)
