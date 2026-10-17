# Examples (spec §9 M8). They use Eyas the way a host does: only the public
# headers (include/eyas), through eyas::eyas.

set(EYAS_UH60A_CONFIG "${PROJECT_SOURCE_DIR}/data/uh60a/uh60a.yaml")

add_executable(eyas_drive drive/eyas_drive.cpp)
target_link_libraries(eyas_drive PRIVATE eyas::eyas)
eyas_target_defaults(eyas_drive)

add_executable(eyas_adapter_example adapter/main.cpp adapter/eyas_adapter.cpp)
target_link_libraries(eyas_adapter_example PRIVATE eyas::eyas)
eyas_target_defaults(eyas_adapter_example)

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
endif()
