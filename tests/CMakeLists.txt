include(GoogleTest)

add_executable(eyas_unit_tests
    support/alloc_counter.cpp
    unit/test_alloc_counter.cpp
    unit/test_axis.cpp
    unit/test_config.cpp
    unit/test_csv_table.cpp
    unit/test_eigen_smoke.cpp
    unit/test_recorder.cpp
    unit/test_table.cpp
    unit/test_units.cpp)
target_include_directories(eyas_unit_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(eyas_unit_tests PRIVATE eyas::eyas GTest::gtest_main)
eyas_target_defaults(eyas_unit_tests)

gtest_discover_tests(eyas_unit_tests DISCOVERY_MODE PRE_TEST)
