# Platform-neutral samples: physics scenes that fill a DrawList, with no SDL, ImGui or fmt. Rendered by a host: the
# SDL executable on PC (main/), the DS samples ROM (nds/samples/).
file(GLOB SAMPLE_FILES CONFIGURE_DEPENDS *.cpp *.h)
add_library(NekoPhysicsSamples STATIC ${SAMPLE_FILES})
# Hosts include "samples/sample.h"
target_include_directories(NekoPhysicsSamples PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/..)
target_link_libraries(NekoPhysicsSamples PUBLIC NekoPhysics)
neko_set_project_warnings(NekoPhysicsSamples)
set_target_properties(NekoPhysicsSamples PROPERTIES FOLDER Samples/)
if(CMAKE_SYSTEM_NAME STREQUAL "NintendoDS")
    target_compile_options(NekoPhysicsSamples PRIVATE -marm -fno-exceptions -fno-rtti)
endif()
