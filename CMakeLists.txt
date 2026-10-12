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
    src/rotor/airfoil.cpp
    src/rotor/blade_element_rotor.cpp
    src/rotor/controls.cpp
    src/rotor/ideal_thrust.cpp
    src/rotor/pitt_peters.cpp
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

# Only include/ is public (D-020). src/ is internal: tests that exercise
# internals add it themselves (eyas_internal below).
target_include_directories(eyas
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    PRIVATE
        ${PROJECT_SOURCE_DIR}/src)
target_link_libraries(eyas
    PUBLIC
        eyas_eigen
    PRIVATE
        yaml-cpp::yaml-cpp)

# Internal headers, for the unit tests and validation generators only.
add_library(eyas_internal INTERFACE)
target_include_directories(eyas_internal INTERFACE ${PROJECT_SOURCE_DIR}/src)
target_link_libraries(eyas_internal INTERFACE eyas yaml-cpp::yaml-cpp)
eyas_target_defaults(eyas)

if(EYAS_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
