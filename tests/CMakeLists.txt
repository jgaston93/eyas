include(GoogleTest)

add_executable(eyas_unit_tests
    support/alloc_counter.cpp
    unit/test_alloc_counter.cpp
    unit/test_atmosphere.cpp
    unit/test_attitude.cpp
    unit/test_axis.cpp
    unit/test_config.cpp
    unit/test_coupled_solve.cpp
    unit/test_csv_table.cpp
    unit/test_eigen_smoke.cpp
    unit/test_executive.cpp
    unit/test_geodesy.cpp
    unit/test_model.cpp
    unit/test_recorder.cpp
    unit/test_rigid_body.cpp
    unit/test_table.cpp
    unit/test_units.cpp)
target_include_directories(eyas_unit_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(eyas_unit_tests PRIVATE eyas_internal GTest::gtest_main)
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
