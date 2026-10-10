include(GoogleTest)

add_executable(eyas_unit_tests
    support/alloc_counter.cpp
    unit/test_alloc_counter.cpp
    unit/test_axis.cpp
    unit/test_config.cpp
    unit/test_coupled_solve.cpp
    unit/test_csv_table.cpp
    unit/test_eigen_smoke.cpp
    unit/test_executive.cpp
    unit/test_recorder.cpp
    unit/test_rigid_body.cpp
    unit/test_table.cpp
    unit/test_units.cpp)
target_include_directories(eyas_unit_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(eyas_unit_tests PRIVATE eyas::eyas GTest::gtest_main)
eyas_target_defaults(eyas_unit_tests)

gtest_discover_tests(eyas_unit_tests DISCOVERY_MODE PRE_TEST)

# Writes the M1 validation time histories (VALIDATION.md; tools/plot_m1.py).
add_executable(eyas_m1_cases validation/m1_cases.cpp)
target_include_directories(eyas_m1_cases PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(eyas_m1_cases PRIVATE eyas::eyas)
eyas_target_defaults(eyas_m1_cases)
