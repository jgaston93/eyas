# Third-party dependencies, fetched at configure time and pinned by version
# and SHA-256. For machines without internet access, point CMake at local
# copies of the same archives' contents, e.g.
#   -DFETCHCONTENT_SOURCE_DIR_EIGEN=/path/to/eigen-3.4.0

include(FetchContent)

# Mark a target's include directories as SYSTEM so its headers cannot trigger
# our -Werror (D-010).
function(eyas_mark_system_includes target)
    get_target_property(dirs ${target} INTERFACE_INCLUDE_DIRECTORIES)
    if(dirs)
        set_target_properties(${target} PROPERTIES INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${dirs}")
    endif()
endfunction()

# --- Eigen 3.4.0 (header-only) ----------------------------------------------
# Eigen's own CMake project (tests, docs, install rules) is not needed, so
# SOURCE_SUBDIR points at a directory without a CMakeLists.txt and we wrap the
# headers in our own INTERFACE target.
FetchContent_Declare(eigen
    URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
    URL_HASH SHA256=8586084f71f9bde545ee7fa6d00288b264a2b7ac3607b974e54d13e7162c1c72
    SOURCE_SUBDIR do-not-configure)

# --- yaml-cpp 0.8.0 (config loading, init time only) --------------------------
set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(YAML_CPP_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
set(YAML_CPP_BUILD_CONTRIB OFF CACHE BOOL "" FORCE)
set(YAML_CPP_INSTALL OFF CACHE BOOL "" FORCE)
set(YAML_BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
FetchContent_Declare(yaml-cpp
    URL https://github.com/jbeder/yaml-cpp/archive/refs/tags/0.8.0.tar.gz
    URL_HASH SHA256=fbe74bbdcee21d656715688706da3c8becfd946d92cd44705cc6098bb23b3a16)

# yaml-cpp 0.8.0 declares cmake_minimum_required(VERSION 3.4); CMake 4.x
# refuses that unless told the project is compatible with 3.5 policies.
set(CMAKE_POLICY_VERSION_MINIMUM 3.5)
FetchContent_MakeAvailable(eigen yaml-cpp)
unset(CMAKE_POLICY_VERSION_MINIMUM)

add_library(eyas_eigen INTERFACE)
target_include_directories(eyas_eigen SYSTEM INTERFACE "${eigen_SOURCE_DIR}")
# Keep Eigen's LGPL-licensed parts out of the build.
target_compile_definitions(eyas_eigen INTERFACE EIGEN_MPL2_ONLY)

eyas_mark_system_includes(yaml-cpp)

# --- stb_easy_font v1.1 (the pilot station's HUD text; examples only) --------
# One public-domain header, pinned by commit and SHA-256. Declared here and
# made available by examples/CMakeLists.txt only when the station's window is
# built (GLFW and OpenGL found).
if(EYAS_BUILD_EXAMPLES)
    FetchContent_Declare(stb_easy_font
        URL https://raw.githubusercontent.com/nothings/stb/2bb4a0accd4003c1db4c24533981e01b1adfd656/stb_easy_font.h
        URL_HASH SHA256=7b48b52a316477766c0a7a096b33a29bcbd847b6fab4fabd32bb3fdb7cea1a60
        DOWNLOAD_NO_EXTRACT TRUE)
endif()

# --- GoogleTest 1.15.2 (tests only) -------------------------------------------
if(EYAS_BUILD_TESTS)
    set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
    set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
    FetchContent_Declare(googletest
        URL https://github.com/google/googletest/archive/refs/tags/v1.15.2.tar.gz
        URL_HASH SHA256=7b42b4d6ed48810c5362c265a17faebe90dc2373c885e5216439d37927f02926)
    FetchContent_MakeAvailable(googletest)
    eyas_mark_system_includes(gtest)
    eyas_mark_system_includes(gtest_main)
endif()
