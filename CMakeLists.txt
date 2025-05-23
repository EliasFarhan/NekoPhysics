
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

cmake_minimum_required (VERSION 3.18)
project(NekoPhysics)

set (CMAKE_CXX_STANDARD 20)
set (CMAKE_CXX_STANDARD_REQUIRED ON)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)


find_package (Python3 COMPONENTS Interpreter)
if(NOT Python3_EXECUTABLE)
    message( FATAL_ERROR "Could not find Python 3" )
endif()
execute_process(COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/scripts/gen_fixed_table.py" WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")




add_subdirectory(externals/)

if(ENABLE_PROFILING)
    find_package(Tracy CONFIG)
endif()

file(GLOB_RECURSE SRC_FILES src/*.cpp include/*.h)

add_library(NekoPhysics ${SRC_FILES} )
target_link_libraries(NekoPhysics PUBLIC unordered_dense)
target_include_directories(NekoPhysics PUBLIC include/ externals/)
link_directories(${CMAKE_BINARY_DIR})
if(ENABLE_PROFILING)
    target_link_libraries(NekoPhysics PUBLIC Tracy::TracyClient)
    target_compile_definitions(NekoPhysics PRIVATE TRACY_ENABLE)
endif()
if (MSVC)
    # warning level 4 and all warnings as errors
    target_compile_options(NekoPhysics PRIVATE /W4 /w14640 /permissive-)
    target_compile_definitions(NekoPhysics PRIVATE _USE_MATH_DEFINES=1)
else()
    # lots of warnings and all warnings as errors
    target_compile_options(NekoPhysics PRIVATE -Wall -Wextra -Wshadow -Wnon-virtual-dtor -Wold-style-cast -Wunused -Werror)
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
        target_include_directories(${TEST_NAME} PRIVATE include/)
        set_target_properties (${TEST_NAME} PROPERTIES FOLDER Tests/)
        gtest_discover_tests(${TEST_NAME})
    endforeach()
endif()