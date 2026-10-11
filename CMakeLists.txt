cmake_minimum_required(VERSION 3.24)

project(eyas
    VERSION 0.1.0
    DESCRIPTION "Real-time rotorcraft flight dynamics model"
    LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(EYAS_BUILD_TESTS "Build the Eyas unit tests" ${PROJECT_IS_TOP_LEVEL})
option(EYAS_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" ON)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
include(EyasCompilerFlags)
include(EyasDependencies)

add_library(eyas STATIC
    src/core/attitude.cpp
    src/core/rigid_body.cpp
    src/env/atmosphere.cpp
    src/env/geodesy.cpp
    src/rotor/ideal_thrust.cpp
    src/util/axis.cpp
    src/util/config.cpp
    src/util/csv_table.cpp
    src/util/parse.cpp
    src/util/table.cpp
    src/util/units.cpp
    src/sim/executive.cpp
    src/sim/model.cpp
    src/sim/recorder.cpp)
add_library(eyas::eyas ALIAS eyas)

target_include_directories(eyas
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>)
target_link_libraries(eyas
    PUBLIC
        eyas_eigen
        yaml-cpp::yaml-cpp)
eyas_target_defaults(eyas)

if(EYAS_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
