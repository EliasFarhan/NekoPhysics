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
if(ENABLE_PROFILING)
    list(APPEND VCPKG_MANIFEST_FEATURES "profiling")
endif (ENABLE_PROFILING)

set(NEKO_SCALAR "FIXED16" CACHE STRING "Type of neko::Scalar: FIXED16 (16.16 fixed point), FLOAT or SIXIT (x86 deterministic float)")
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

file(GLOB_RECURSE SRC_FILES src/*.cpp include/*.h)

add_library(NekoPhysics ${SRC_FILES} )
target_include_directories(NekoPhysics PUBLIC include/)
target_include_directories(NekoPhysics SYSTEM PUBLIC externals/)
link_directories(${CMAKE_BINARY_DIR})
if(ENABLE_PROFILING)
    target_link_libraries(NekoPhysics PUBLIC Tracy::TracyClient)
    target_compile_definitions(NekoPhysics PRIVATE TRACY_ENABLE)
endif()
target_compile_definitions(NekoPhysics PUBLIC NEKO_SCALAR_${NEKO_SCALAR})
message(STATUS "NekoPhysics: neko::Scalar is ${NEKO_SCALAR}")
neko_set_project_warnings(NekoPhysics)
if (MSVC)
    target_compile_definitions(NekoPhysics PRIVATE _USE_MATH_DEFINES=1)
endif()
# devkitPro's NDS.cmake sets CMAKE_SYSTEM_NAME to NintendoDS (and NINTENDO_DS, ARM9 or ARM7)
if(CMAKE_SYSTEM_NAME STREQUAL "NintendoDS")
    target_compile_definitions(NekoPhysics PUBLIC NEKO_PLATFORM_NDS)
    # In Thumb mode a Fixed16 multiply is a __aeabi_lmul call, in ARM mode it is an inline SMULL
    target_compile_options(NekoPhysics PRIVATE -marm)
endif()

#install(TARGETS NekoPhysics FILE_SET HEADERS)

if(USE_SAMPLES)
    find_package(box2d CONFIG REQUIRED)
    find_package(SDL3 CONFIG REQUIRED)
    find_package(imgui CONFIG REQUIRED)
    find_package(fmt CONFIG REQUIRED)
    file(GLOB_RECURSE MAIN_FILES main/*cpp main/*.h)
    add_executable(Sample ${MAIN_FILES})
    target_include_directories(Sample PRIVATE include/ main/ ${OPENGL_INCLUDE_DIR})
    target_link_libraries(Sample PRIVATE NekoPhysics ${OPENGL_LIBRARIES} fmt::fmt-header-only SDL3::SDL3
            imgui::imgui box2d::box2d)
    neko_set_project_warnings(Sample)
    if(MSVC)
        target_link_libraries(Sample PRIVATE opengl32)
    endif()
endif()

if(USE_TESTING)
    find_package(GTest CONFIG REQUIRED)
    file(GLOB_RECURSE TEST_FILES tests/*.cpp)
    enable_testing()
    include(GoogleTest)
    foreach(TEST_FILE ${TEST_FILES})
        get_filename_component(TEST_NAME ${TEST_FILE} NAME_WE )
        add_executable(${TEST_NAME} ${TEST_FILE})
        target_link_libraries(${TEST_NAME} NekoPhysics GTest::gtest GTest::gmock)
        target_include_directories(${TEST_NAME} PRIVATE include/ tests/)
        neko_set_project_warnings(${TEST_NAME})
        set_target_properties (${TEST_NAME} PROPERTIES FOLDER Tests/)
        gtest_discover_tests(${TEST_NAME})
    endforeach()
endif()

if(CMAKE_SYSTEM_NAME STREQUAL "NintendoDS")
    add_subdirectory(nds)
endif()
