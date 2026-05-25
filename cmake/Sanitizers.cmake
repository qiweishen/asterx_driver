#
# Sanitizer wiring for asterx_driver.
#
# Usage:
#     asterx_set_sanitizers(<target>)
#
# Options (top-level cache variables, see CMakeLists.txt):
#   ASTERX_ENABLE_ASAN  -- AddressSanitizer
#   ASTERX_ENABLE_UBSAN -- UndefinedBehaviorSanitizer
#   ASTERX_ENABLE_TSAN  -- ThreadSanitizer
#
# Mutual exclusivity rules (enforced at configure time):
#   * ASAN and TSAN cannot both be ON.
#   * TSAN cannot be combined with ASAN.
#   * UBSAN can combine freely with ASAN or TSAN.
#
# Sanitizers are runtime overhead; we refuse to enable them under Release.
#

include_guard(GLOBAL)

# Mutual exclusivity check (run once at first include).
if(ASTERX_ENABLE_ASAN AND ASTERX_ENABLE_TSAN)
    message(FATAL_ERROR
        "ASTERX_ENABLE_ASAN and ASTERX_ENABLE_TSAN are mutually exclusive. "
        "Pick one.")
endif()

# Release-build guard: warn loudly, then refuse to apply sanitizers.
if((ASTERX_ENABLE_ASAN OR ASTERX_ENABLE_UBSAN OR ASTERX_ENABLE_TSAN)
    AND CMAKE_BUILD_TYPE STREQUAL "Release")
    message(WARNING
        "Sanitizers are enabled on a Release build. They will be silently "
        "skipped. Switch to -DCMAKE_BUILD_TYPE=Debug or RelWithDebInfo if "
        "you want sanitizers compiled in.")
    set(ASTERX_SANITIZERS_ACTIVE OFF)
elseif(ASTERX_ENABLE_ASAN OR ASTERX_ENABLE_UBSAN OR ASTERX_ENABLE_TSAN)
    set(ASTERX_SANITIZERS_ACTIVE ON)
else()
    set(ASTERX_SANITIZERS_ACTIVE OFF)
endif()

function(asterx_set_sanitizers target)
    if(NOT TARGET ${target})
        message(FATAL_ERROR "asterx_set_sanitizers: target '${target}' does not exist.")
    endif()

    if(NOT ASTERX_SANITIZERS_ACTIVE)
        return()
    endif()

    if(MSVC)
        message(WARNING "asterx_set_sanitizers: MSVC sanitizer flags are not implemented; skipping.")
        return()
    endif()

    set(_flags "")

    if(ASTERX_ENABLE_ASAN)
        list(APPEND _flags
            -fsanitize=address
            -fno-omit-frame-pointer
            -fno-optimize-sibling-calls)
    endif()

    if(ASTERX_ENABLE_UBSAN)
        list(APPEND _flags
            -fsanitize=undefined
            -fno-sanitize-recover=undefined)
    endif()

    if(ASTERX_ENABLE_TSAN)
        list(APPEND _flags
            -fsanitize=thread)
    endif()

    target_compile_options(${target} PRIVATE ${_flags})
    target_link_options(${target}    PRIVATE ${_flags})
endfunction()
