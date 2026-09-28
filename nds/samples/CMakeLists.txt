# DS samples: NdsSampleDrawer renders the platform-neutral NekoPhysicsSamples on the top screen (hardware sprites and
# a BG3 8bpp bitmap)
add_library(NekoPhysicsDsSampleDrawer STATIC nds_sample_drawer.cpp nds_sample_drawer.h)
target_include_directories(NekoPhysicsDsSampleDrawer PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(NekoPhysicsDsSampleDrawer PUBLIC NekoPhysicsSamples)
target_compile_options(NekoPhysicsDsSampleDrawer PRIVATE -marm -fno-exceptions -fno-rtti)
neko_set_project_warnings(NekoPhysicsDsSampleDrawer)

add_executable(NekoPhysicsDsSamples main.cpp)
target_link_libraries(NekoPhysicsDsSamples PRIVATE NekoPhysicsDsSampleDrawer)
target_compile_options(NekoPhysicsDsSamples PRIVATE -marm -fno-exceptions -fno-rtti)
target_link_options(NekoPhysicsDsSamples PRIVATE -Wl,-Map,$<TARGET_FILE_DIR:NekoPhysicsDsSamples>/NekoPhysicsDsSamples.map)
neko_set_project_warnings(NekoPhysicsDsSamples)
nds_create_rom(NekoPhysicsDsSamples NAME "NekoPhysics" SUBTITLE1 "Samples" SUBTITLE2 "sprites and bitmap")
