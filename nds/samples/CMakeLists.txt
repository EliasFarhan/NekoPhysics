# DS samples: NdsSampleDrawer renders the platform-neutral NekoPhysicsSamples on the top screen (hardware sprites and
# a BG3 8bpp bitmap)
add_library(NekoPhysicsDsSampleDrawer STATIC nds_sample_drawer.cpp nds_sample_drawer.h)
target_include_directories(NekoPhysicsDsSampleDrawer PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(NekoPhysicsDsSampleDrawer PUBLIC NekoPhysicsSamples)
target_compile_options(NekoPhysicsDsSampleDrawer PRIVATE -marm -fno-exceptions -fno-rtti)
neko_set_project_warnings(NekoPhysicsDsSampleDrawer)
