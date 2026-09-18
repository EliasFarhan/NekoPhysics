# Warning set adapted from cpp-best-practices (Jason Turner).
# Usage: neko_set_project_warnings(<target> [PRIVATE|INTERFACE])

# TODO: flip to ON once the existing sign-conversion (C4365 / -Wsign-conversion) warnings are fixed.
option(NEKO_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)

function(neko_set_project_warnings target)
    set(scope PRIVATE)
    if(ARGC GREATER 1)
        set(scope ${ARGV1})
    endif()

    set(MSVC_WARNINGS
        /W4
        /permissive-
        /w14242 # 'identifier': conversion from 'type1' to 'type2', possible loss of data
        /w14254 # 'operator': conversion from 'type1:field_bits' to 'type2:field_bits', possible loss of data
        /w14263 # member function does not override any base class virtual member function
        /w14265 # class has virtual functions, but destructor is not virtual
        /w14287 # unsigned/negative constant mismatch
        /we4289 # loop control variable used outside the for-loop scope
        /w14296 # expression is always true/false
        /w14311 # pointer truncation
        /w14545 # expression before comma evaluates to a function missing an argument list
        /w14546 # function call before comma missing argument list
        /w14547 # operator before comma has no effect
        /w14549 # operator before comma has no effect
        /w14555 # expression has no effect
        /w14619 # pragma warning: there is no warning number 'number'
        /w14640 # thread-unsafe static member initialization
        /w14826 # conversion is sign-extended
        /w14905 # wide string literal cast to 'LPSTR'
        /w14906 # string literal cast to 'LPWSTR'
        /w14928 # illegal copy-initialization
        /w14365 # signed/unsigned mismatch on conversion
    )

    set(CLANG_WARNINGS
        -Wall
        -Wextra
        -Wpedantic
        -Wshadow
        -Wnon-virtual-dtor
        -Wold-style-cast
        -Wcast-align
        -Wunused
        -Woverloaded-virtual
        -Wconversion
        -Wsign-conversion
        -Wnull-dereference
        -Wdouble-promotion
        -Wformat=2
        -Wimplicit-fallthrough
        -Wextra-semi
    )

    set(GCC_WARNINGS
        ${CLANG_WARNINGS}
        -Wmisleading-indentation
        -Wduplicated-cond
        -Wduplicated-branches
        -Wlogical-op
        -Wuseless-cast
        -Wsuggest-override
    )

    if(NEKO_WARNINGS_AS_ERRORS)
        list(APPEND MSVC_WARNINGS /WX)
        list(APPEND CLANG_WARNINGS -Werror)
        list(APPEND GCC_WARNINGS -Werror)
    endif()

    if(MSVC)
        # Also covers clang-cl, which understands the MSVC spelling.
        set(warnings ${MSVC_WARNINGS})
    elseif(CMAKE_CXX_COMPILER_ID MATCHES ".*Clang")
        set(warnings ${CLANG_WARNINGS})
    elseif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        set(warnings ${GCC_WARNINGS})
    else()
        message(AUTHOR_WARNING "No compiler warnings set for '${CMAKE_CXX_COMPILER_ID}'")
    endif()

    target_compile_options(${target} ${scope} ${warnings})
endfunction()
