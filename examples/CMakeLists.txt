# Examples (spec §9 M8). They use Eyas the way a host does: only the public
# headers (include/eyas), through eyas::eyas.

set(EYAS_UH60A_CONFIG "${PROJECT_SOURCE_DIR}/data/uh60a/uh60a.yaml")

# Shared by the driver and the pilot station: eyas_drive's control files and
# state CSV, and the --origin parser.
add_library(eyas_examples_common STATIC common/control_file.cpp common/state_csv.cpp)
target_include_directories(eyas_examples_common PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(eyas_examples_common PUBLIC eyas::eyas)
eyas_target_defaults(eyas_examples_common)

add_executable(eyas_drive drive/eyas_drive.cpp)
target_link_libraries(eyas_drive PRIVATE eyas_examples_common)
eyas_target_defaults(eyas_drive)

add_executable(eyas_adapter_example adapter/main.cpp adapter/eyas_adapter.cpp)
target_link_libraries(eyas_adapter_example PRIVATE eyas::eyas)
eyas_target_defaults(eyas_adapter_example)

# --- Pilot station (M8b, D-044) ----------------------------------------------
# The core (mapping, FlightGear link, start points, recorder) has no window,
# so the tests can drive it; the GLFW/OpenGL window is added when both are
# found, and the station runs --headless otherwise.
add_library(eyas_station_core STATIC
    pilot_station/controls_map.cpp
    pilot_station/mapping.cpp
    pilot_station/fg_link.cpp
    pilot_station/station.cpp
    pilot_station/recorder.cpp)
target_include_directories(eyas_station_core PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/pilot_station")
target_link_libraries(eyas_station_core PUBLIC eyas_examples_common)
eyas_target_defaults(eyas_station_core)

add_executable(eyas_pilot_station pilot_station/main.cpp)
target_link_libraries(eyas_pilot_station PRIVATE eyas_station_core)
eyas_target_defaults(eyas_pilot_station)

find_package(glfw3 3.3 CONFIG QUIET)
set(OpenGL_GL_PREFERENCE GLVND)
find_package(OpenGL QUIET)
if(glfw3_FOUND AND OpenGL_FOUND)
    FetchContent_MakeAvailable(stb_easy_font)
    target_sources(eyas_pilot_station PRIVATE pilot_station/gl_view.cpp)
    target_include_directories(eyas_pilot_station SYSTEM PRIVATE "${stb_easy_font_SOURCE_DIR}")
    target_link_libraries(eyas_pilot_station PRIVATE glfw OpenGL::GL)
    target_compile_definitions(eyas_pilot_station PRIVATE EYAS_STATION_GL)
    message(STATUS "Eyas pilot station: with the GLFW/OpenGL window")
else()
    message(STATUS "Eyas pilot station: headless only (GLFW 3.3 and OpenGL not both found)")
endif()

if(EYAS_BUILD_TESTS)
    # Smoke tests: the driver flies the 10-s doublet from a 60 kt trim
    # (exit 0: every frame good), and its output has the header and one row
    # at t = 0 plus one per 60 Hz frame.
    set(doublet_out "${CMAKE_CURRENT_BINARY_DIR}/doublet_10s_out.csv")
    add_test(NAME ExampleDrive.Doublet10s
        COMMAND eyas_drive --config ${EYAS_UH60A_CONFIG}
                --controls ${CMAKE_CURRENT_SOURCE_DIR}/drive/doublet_10s.csv
                --out ${doublet_out} --trim-speed 60)
    set_tests_properties(ExampleDrive.Doublet10s PROPERTIES FIXTURES_SETUP doublet_out)
    add_test(NAME ExampleDrive.Doublet10sOutput
        COMMAND ${CMAKE_COMMAND} -DFILE=${doublet_out} -DLINES=602
                -P ${CMAKE_CURRENT_SOURCE_DIR}/drive/check_output.cmake)
    set_tests_properties(ExampleDrive.Doublet10sOutput PROPERTIES FIXTURES_REQUIRED doublet_out)

    add_test(NAME ExampleAdapter.Flies5s COMMAND eyas_adapter_example ${EYAS_UH60A_CONFIG})

    # The station's recording replays exactly: it flies the doublet headless
    # at another origin and writes its state history and a recording; then
    # eyas_drive replays the recording with the command in its header, and
    # the two state histories must be byte-identical (D-044).
    add_test(NAME ExampleStation.RecordingReplaysExactly
        COMMAND ${CMAKE_COMMAND}
                -DSTATION=$<TARGET_FILE:eyas_pilot_station> -DDRIVE=$<TARGET_FILE:eyas_drive>
                -DCONFIG=${EYAS_UH60A_CONFIG}
                -DCONTROLS=${CMAKE_CURRENT_SOURCE_DIR}/drive/doublet_10s.csv
                -DWORK=${CMAKE_CURRENT_BINARY_DIR}/station_replay
                -P ${CMAKE_CURRENT_SOURCE_DIR}/pilot_station/check_replay.cmake)

    add_executable(eyas_station_tests
        pilot_station/tests/test_station.cpp
        ${PROJECT_SOURCE_DIR}/tests/support/alloc_counter.cpp)
    target_include_directories(eyas_station_tests PRIVATE "${PROJECT_SOURCE_DIR}/tests")
    target_link_libraries(eyas_station_tests PRIVATE eyas_station_core GTest::gtest_main)
    target_compile_definitions(eyas_station_tests PRIVATE
        EYAS_UH60A_CONFIG="${EYAS_UH60A_CONFIG}"
        EYAS_STATION_FG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/pilot_station/flightgear")
    eyas_target_defaults(eyas_station_tests)
    include(GoogleTest)
    gtest_discover_tests(eyas_station_tests DISCOVERY_MODE PRE_TEST)
endif()
