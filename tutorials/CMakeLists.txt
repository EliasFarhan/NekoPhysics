# Tutorial chapters of the NekoPhysics articles (GPR_SAE_Slides/nekophysics/). Each chapter folder NN_<slug>/ holds a
# platform-neutral Sample (<slug>_sample.h/.cc, in NekoPhysicsTutorials, tested by tests/tutorials_test.cc) and a
# main.cc running it on the PC SDL host as course_NN_<slug>. Exercise solutions will be course_NN_<slug>_<name>.
file(GLOB TUTORIAL_SAMPLE_FILES CONFIGURE_DEPENDS */*_sample.cc */*_sample.h)
add_library(NekoPhysicsTutorials STATIC ${TUTORIAL_SAMPLE_FILES})
target_include_directories(NekoPhysicsTutorials PUBLIC ${PROJECT_SOURCE_DIR})
target_link_libraries(NekoPhysicsTutorials PUBLIC NekoPhysicsSamples)
neko_set_project_warnings(NekoPhysicsTutorials)
set_target_properties(NekoPhysicsTutorials PROPERTIES FOLDER Tutorials/)

# neko_add_tutorial(<NN_slug>): the chapter program course_<NN_slug>, from <NN_slug>/main.cc
function(neko_add_tutorial chapter)
    add_executable(course_${chapter} ${chapter}/main.cc)
    target_link_libraries(course_${chapter} PRIVATE NekoPhysicsTutorials NekoPhysicsSdlHost)
    neko_set_project_warnings(course_${chapter})
    set_target_properties(course_${chapter} PROPERTIES FOLDER Tutorials/)
endfunction()

if(USE_SAMPLES)
    neko_add_tutorial(00_bounce)
    neko_add_tutorial(01_rotation)
    neko_add_tutorial(02_rotated_box)
    neko_add_tutorial(03_contact_manifold)
    neko_add_tutorial(04_off_center_impact)
endif()
