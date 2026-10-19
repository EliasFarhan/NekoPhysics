cmake_minimum_required (VERSION 3.18)


option(USE_SAMPLES "Use Samples" OFF)
if(USE_SAMPLES)
    list(APPEND VCPKG_MANIFEST_FEATURES "samples")
endif (USE_SAMPLES)
option(USE_TESTING "Use Testing" OFF)
if(USE_TESTING)
    list(APPEND VCPKG_MANIFEST_FEATURES "tests")
endif (USE_TESTING)

option(ENABLE_PROFILING "Use Tracy Profiler" OFF)
option(NEKO_BUILD_DOCS "Add the docs target (API reference with MrDocs)" OFF)
if(ENABLE_PROFILING)
    list(APPEND VCPKG_MANIFEST_FEATURES "profiling")
endif (ENABLE_PROFILING)

set(NEKO_SCALAR "FIXED16" CACHE STRING "Type of neko::Scalar: FIXED16 (16.16 fixed point), FLOAT or SIXIT (sixit deterministic float: x86, x64, RISC-V, arm64 with GCC or clang)")
set_property(CACHE NEKO_SCALAR PROPERTY STRINGS FIXED16 FLOAT SIXIT)
if(NOT NEKO_SCALAR MATCHES "^(FIXED16|FLOAT|SIXIT)$")
    message(FATAL_ERROR "NEKO_SCALAR must be FIXED16, FLOAT or SIXIT (got '${NEKO_SCALAR}')")
endif()

project(NekoPhysics)

set (CMAKE_CXX_STANDARD 23)
set (CMAKE_CXX_STANDARD_REQUIRED ON)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(CompilerWarnings)






add_subdirectory(externals/)

if(ENABLE_PROFILING)
    find_package(Tracy CONFIG)
endif()

file(GLOB_RECURSE SRC_FILES src/*.cc include/*.h)

add_library(NekoPhysics ${SRC_FILES} )
target_include_directories(NekoPhysics PUBLIC include/)
target_include_directories(NekoPhysics SYSTEM PUBLIC externals/)
link_directories(${CMAKE_BINARY_DIR})
if(ENABLE_PROFILING)
    target_link_libraries(NekoPhysics PUBLIC Tracy::TracyClient)
    target_compile_definitions(NekoPhysics PRIVATE TRACY_ENABLE)
endif()
target_compile_definitions(NekoPhysics PUBLIC NEKO_SCALAR_${NEKO_SCALAR})
if(NOT MSVC)
    # No fused multiply-add: clang contracts a * b + c by default on arm64 (and
    # GCC/clang on x64 with -march=...+fma), so a float build would give other
    # bits there. PUBLIC because the headers inline the Vec2 math into the
    # including files. MSVC doesn't contract by default (/fp:precise)
    target_compile_options(NekoPhysics PUBLIC -ffp-contract=off)
endif()
message(STATUS "NekoPhysics: neko::Scalar is ${NEKO_SCALAR}")
neko_set_project_warnings(NekoPhysics)
if (MSVC)
    target_compile_definitions(NekoPhysics PRIVATE _USE_MATH_DEFINES=1)
else()
    # No exceptions or RTTI in the library (Google style); the tests keep them for GoogleTest
    target_compile_options(NekoPhysics PRIVATE -fno-exceptions -fno-rtti)
endif()
# devkitPro's NDS.cmake sets CMAKE_SYSTEM_NAME to NintendoDS (and NINTENDO_DS, ARM9 or ARM7)
if(CMAKE_SYSTEM_NAME STREQUAL "NintendoDS")
    target_compile_definitions(NekoPhysics PUBLIC NEKO_PLATFORM_NDS)
    # In Thumb mode a Fixed16 multiply is a __aeabi_lmul call, in ARM mode it is an inline SMULL
    target_compile_options(NekoPhysics PRIVATE -marm)
endif()

#install(TARGETS NekoPhysics FILE_SET HEADERS)

# The DS builds the samples library without USE_SAMPLES: the nds preset keeps it OFF to avoid vcpkg's SDL.
# tests/samples_test.cc links it too.
if(USE_SAMPLES OR USE_TESTING OR CMAKE_SYSTEM_NAME STREQUAL "NintendoDS")
    add_subdirectory(main/samples)
endif()

if(USE_SAMPLES)
    find_package(box2d CONFIG REQUIRED)
    find_package(SDL3 CONFIG REQUIRED)
    find_package(imgui CONFIG REQUIRED)
    # PC host (SDL window, ImGui menu, SdlSampleDrawer), shared by the Sample executable and the tutorial programs
    add_library(NekoPhysicsSdlHost STATIC main/sdl/sdl_host.cc main/sdl/sdl_host.h
        main/sdl/sdl_sample_drawer.cc main/sdl/sdl_sample_drawer.h)
    target_include_directories(NekoPhysicsSdlHost PUBLIC ${PROJECT_SOURCE_DIR})
    target_link_libraries(NekoPhysicsSdlHost PUBLIC NekoPhysicsSamples SDL3::SDL3 imgui::imgui)
    neko_set_project_warnings(NekoPhysicsSdlHost)
    set_target_properties(NekoPhysicsSdlHost PROPERTIES FOLDER Samples/)
    if(MSVC)
        target_link_libraries(NekoPhysicsSdlHost PUBLIC opengl32)
    endif()
    add_executable(Sample main/sdl/main.cc)
    target_link_libraries(Sample PRIVATE NekoPhysicsSdlHost box2d::box2d)
    neko_set_project_warnings(Sample)
    set_target_properties(Sample PROPERTIES FOLDER Samples/)
endif()

# Tutorial chapters (one program each on the SDL host), used by the NekoPhysics articles
if(USE_SAMPLES OR USE_TESTING)
    add_subdirectory(tutorials)
endif()

if(USE_SAMPLES OR USE_TESTING)
    # Checksum stream of every sample and tutorial, compared across platforms
    # (RunSampleStream in a library, so another host can call it)
    add_library(NekoPhysicsSampleStream STATIC main/stream/sample_stream.cc main/stream/sample_stream.h)
    target_link_libraries(NekoPhysicsSampleStream PUBLIC NekoPhysicsTutorials)
    target_include_directories(NekoPhysicsSampleStream PUBLIC ${PROJECT_SOURCE_DIR})
    neko_set_project_warnings(NekoPhysicsSampleStream)
    set_target_properties(NekoPhysicsSampleStream PROPERTIES FOLDER Samples/)
    add_executable(sample_stream main/stream/sample_stream_main.cc)
    target_link_libraries(sample_stream PRIVATE NekoPhysicsSampleStream)
    neko_set_project_warnings(sample_stream)
    set_target_properties(sample_stream PROPERTIES FOLDER Samples/)
endif()

if(USE_TESTING)
    find_package(GTest CONFIG REQUIRED)
    file(GLOB_RECURSE TEST_FILES tests/*.cc)
    enable_testing()
    include(GoogleTest)
    foreach(TEST_FILE ${TEST_FILES})
        get_filename_component(TEST_NAME ${TEST_FILE} NAME_WE )
        add_executable(${TEST_NAME} ${TEST_FILE})
        target_link_libraries(${TEST_NAME} NekoPhysics GTest::gtest GTest::gmock)
        if(TEST_NAME STREQUAL "samples_test")
            target_link_libraries(${TEST_NAME} NekoPhysicsSamples)
        endif()
        if(TEST_NAME STREQUAL "tutorials_test")
            target_link_libraries(${TEST_NAME} NekoPhysicsTutorials)
        endif()
        target_include_directories(${TEST_NAME} PRIVATE ${PROJECT_SOURCE_DIR})
        neko_set_project_warnings(${TEST_NAME})
        set_target_properties (${TEST_NAME} PROPERTIES FOLDER Tests/)
        gtest_discover_tests(${TEST_NAME})
    endforeach()
    add_test(NAME SampleStream COMMAND sample_stream 10)
endif()

if(CMAKE_SYSTEM_NAME STREQUAL "NintendoDS")
    add_subdirectory(nds)
endif()

if(NEKO_BUILD_DOCS)
    add_subdirectory(docs)
endif()
