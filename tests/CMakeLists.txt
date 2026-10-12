include(GoogleTest)

add_executable(eyas_unit_tests
    support/alloc_counter.cpp
    unit/test_alloc_counter.cpp
    unit/test_atmosphere.cpp
    unit/test_attitude.cpp
    unit/test_axis.cpp
    unit/test_blade_element_rotor.cpp
    unit/test_config.cpp
    unit/test_coupled_solve.cpp
    unit/test_csv_table.cpp
    unit/test_eigen_smoke.cpp
    unit/test_executive.cpp
    unit/test_flap_anchors.cpp
    unit/test_flapping_rotor.cpp
    unit/test_geodesy.cpp
    unit/test_model.cpp
    unit/test_pitt_peters.cpp
    unit/test_recorder.cpp
    unit/test_rotor_inputs.cpp
    unit/test_rigid_body.cpp
    unit/test_table.cpp
    unit/test_units.cpp
    unit/test_uh60a_data.cpp)
target_include_directories(eyas_unit_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(eyas_unit_tests PRIVATE eyas_internal GTest::gtest_main)
target_compile_definitions(eyas_unit_tests PRIVATE EYAS_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data"
    EYAS_VEHICLE_DATA_DIR="${PROJECT_SOURCE_DIR}/data")
eyas_target_defaults(eyas_unit_tests)

gtest_discover_tests(eyas_unit_tests DISCOVERY_MODE PRE_TEST)

# Built the way a host builds: include/eyas only, no src/ on the include path,
# so an internal header leaking into the API fails to compile here (D-020).
add_executable(eyas_api_tests api/test_public_api.cpp)
target_link_libraries(eyas_api_tests PRIVATE eyas::eyas GTest::gtest_main)
eyas_target_defaults(eyas_api_tests)

gtest_discover_tests(eyas_api_tests DISCOVERY_MODE PRE_TEST)

# Writes the M1 validation time histories (VALIDATION.md; tools/plot_m1.py).
add_executable(eyas_m1_cases validation/m1_cases.cpp)
target_include_directories(eyas_m1_cases PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(eyas_m1_cases PRIVATE eyas_internal)
eyas_target_defaults(eyas_m1_cases)

# Writes the M2 validation data (VALIDATION.md; tools/plot_m2.py).
add_executable(eyas_m2_cases validation/m2_cases.cpp)
target_include_directories(eyas_m2_cases PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(eyas_m2_cases PRIVATE eyas_internal)
eyas_target_defaults(eyas_m2_cases)

# Writes the M3 validation data (VALIDATION.md; tools/plot_m3.py).
add_executable(eyas_m3_cases validation/m3_cases.cpp)
target_include_directories(eyas_m3_cases PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(eyas_m3_cases PRIVATE eyas_internal)
eyas_target_defaults(eyas_m3_cases)

# Writes the M4 validation data (VALIDATION.md; tools/plot_m4.py).
add_executable(eyas_m4_cases validation/m4_cases.cpp)
target_include_directories(eyas_m4_cases PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(eyas_m4_cases PRIVATE eyas_internal)
eyas_target_defaults(eyas_m4_cases)

# Writes the M5 validation data (VALIDATION.md; tools/plot_m5.py).
add_executable(eyas_m5_cases validation/m5_cases.cpp)
target_include_directories(eyas_m5_cases PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(eyas_m5_cases PRIVATE eyas_internal)
eyas_target_defaults(eyas_m5_cases)
