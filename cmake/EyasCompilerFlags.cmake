# Compiler settings for Eyas targets (DECISIONS.md D-010).
#
# - Strict warnings on our own code only; third-party headers are SYSTEM
#   includes (see EyasDependencies.cmake) so their warnings cannot fail the
#   build.
# - No fast-math, and FP contraction (FMA fusion) explicitly off, so results
#   are bit-identical for a given build configuration and compiler.

if(NOT CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    message(FATAL_ERROR "Eyas supports GCC and Clang only (got ${CMAKE_CXX_COMPILER_ID})")
endif()

# C++20 library features used (<numbers>, <span>, floating-point
# std::from_chars) need GCC 11 or Clang 14. RHEL 8's system GCC is 8.5.
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 11)
    message(FATAL_ERROR
        "Eyas needs GCC >= 11 (found ${CMAKE_CXX_COMPILER_VERSION}). On RHEL 8:\n"
        "  dnf install gcc-toolset-13-gcc-c++ && source /opt/rh/gcc-toolset-13/enable\n"
        "then configure in a fresh build directory.")
endif()
if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 14)
    message(FATAL_ERROR "Eyas needs Clang >= 14 (found ${CMAKE_CXX_COMPILER_VERSION})")
endif()

foreach(flags_var IN ITEMS CMAKE_CXX_FLAGS CMAKE_CXX_FLAGS_DEBUG CMAKE_CXX_FLAGS_RELEASE
                           CMAKE_CXX_FLAGS_RELWITHDEBINFO CMAKE_CXX_FLAGS_MINSIZEREL)
    if("${${flags_var}}" MATCHES "fast-math|-Ofast|finite-math-only|unsafe-math-optimizations")
        message(FATAL_ERROR "${flags_var} enables fast-math; forbidden by D-010 (determinism)")
    endif()
endforeach()

function(eyas_target_defaults target)
    target_compile_options(${target} PRIVATE
        -Wall
        -Wextra
        -Wpedantic
        -Wshadow
        -Wconversion
        -Wsign-conversion
        -Wdouble-promotion
        -Wold-style-cast
        -Wnon-virtual-dtor
        -Woverloaded-virtual
        -Wimplicit-fallthrough
        -fno-fast-math
        -ffp-contract=off
        # Eigen is ~150x slower at -O0; -Og keeps debugging and assertions
        # usable while long-running tests stay fast (D-010).
        # EYAS_DEBUG_OPTIMIZATION (default -Og) sets it.
        $<$<CONFIG:Debug>:${EYAS_DEBUG_OPTIMIZATION}>)
    if(EYAS_WARNINGS_AS_ERRORS)
        target_compile_options(${target} PRIVATE -Werror)
    endif()
endfunction()
