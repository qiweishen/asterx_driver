#
# Compiler warning policy for asterx_driver.
#
# Usage:
#     asterx_set_warnings(<target>)
#
# Applies a moderately strict warning set tuned for GCC 9.4+ and Clang 10+.
# Warnings-as-errors is opt-in via the cache variable ASTERX_WARNINGS_AS_ERRORS.
#

include_guard(GLOBAL)

set(ASTERX_GCC_WARNINGS
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
    -Wmisleading-indentation
    -Wduplicated-cond
    -Wduplicated-branches
    -Wlogical-op
    -Wnull-dereference
    -Wdouble-promotion
    -Wformat=2
    -Wimplicit-fallthrough
)

set(ASTERX_CLANG_WARNINGS
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
)

set(ASTERX_MSVC_WARNINGS
    /W4
    /permissive-
)

function(asterx_set_warnings target)
    if(NOT TARGET ${target})
        message(FATAL_ERROR "asterx_set_warnings: target '${target}' does not exist.")
    endif()

    if(MSVC)
        target_compile_options(${target} PRIVATE ${ASTERX_MSVC_WARNINGS})
        if(ASTERX_WARNINGS_AS_ERRORS)
            target_compile_options(${target} PRIVATE /WX)
        endif()
    elseif(CMAKE_CXX_COMPILER_ID STREQUAL "Clang"
        OR CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang")
        target_compile_options(${target} PRIVATE ${ASTERX_CLANG_WARNINGS})
        if(ASTERX_WARNINGS_AS_ERRORS)
            target_compile_options(${target} PRIVATE -Werror)
        endif()
    elseif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        target_compile_options(${target} PRIVATE ${ASTERX_GCC_WARNINGS})
        if(ASTERX_WARNINGS_AS_ERRORS)
            target_compile_options(${target} PRIVATE -Werror)
        endif()
    else()
        message(WARNING "asterx_set_warnings: unknown compiler '${CMAKE_CXX_COMPILER_ID}'; skipping.")
    endif()
endfunction()
