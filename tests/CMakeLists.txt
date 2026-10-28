include(GoogleTest)

add_executable(eyas_unit_tests
    support/alloc_counter.cpp
    unit/test_alloc_counter.cpp
    unit/test_airframe_aero.cpp
    unit/test_spinning_parts.cpp
    unit/test_kaplita_sideslip.cpp
    unit/test_air_data.cpp
    unit/test_atmosphere.cpp
    unit/test_bailey_rotor.cpp
    unit/test_attitude.cpp
    unit/test_ballin_trim.cpp
    unit/test_axis.cpp
    unit/test_blade_element_rotor.cpp
    unit/test_config.cpp
    unit/test_coupled_solve.cpp
    unit/test_coupling.cpp
    unit/test_dynamic_twist.cpp
    unit/test_csv_table.cpp
    unit/test_eigen_smoke.cpp
    unit/test_executive.cpp
    unit/test_trim.cpp
    unit/test_linearize.cpp
    unit/test_flap_anchors.cpp
    unit/test_flapping_rotor.cpp
    unit/test_genhel_fcs.cpp
    unit/test_genhel_sas.cpp
    unit/test_genhel_fps.cpp
    unit/test_ballin_t700.cpp
    unit/test_genhel_t700.cpp
    unit/test_uh60a_fps.cpp
    unit/test_geodesy.cpp
    unit/test_landing_gear.cpp
    unit/test_model.cpp
    unit/test_outputs.cpp
    unit/test_readouts.cpp
    unit/test_pitt_peters.cpp
    unit/test_vortex_ring.cpp
    unit/test_ground_effect.cpp
    unit/test_shafts.cpp
    unit/test_realtime.cpp
    unit/test_recorder.cpp
    unit/test_rotor_inputs.cpp
    unit/test_rigid_body.cpp
    unit/test_table.cpp
    unit/test_tail_rotor_wake.cpp
    unit/test_units.cpp
    unit/test_yawed_flow.cpp
    unit/test_uh60a_data.cpp
    unit/test_uh60a_aircraft.cpp)
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

# Writes the M6 validation data (VALIDATION.md; tools/plot_m6.py).
add_executable(eyas_m6_cases validation/m6_cases.cpp)
target_include_directories(eyas_m6_cases PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(eyas_m6_cases PRIVATE eyas_internal)
target_compile_definitions(eyas_m6_cases PRIVATE EYAS_VEHICLE_DATA_DIR="${PROJECT_SOURCE_DIR}/data")
eyas_target_defaults(eyas_m6_cases)

# Writes the M6c landing-gear validation data (VALIDATION.md; tools/plot_m6c.py).
add_executable(eyas_m6c_cases validation/m6c_cases.cpp)
target_include_directories(eyas_m6c_cases PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(eyas_m6c_cases PRIVATE eyas_internal)
target_compile_definitions(eyas_m6c_cases PRIVATE EYAS_VEHICLE_DATA_DIR="${PROJECT_SOURCE_DIR}/data")
eyas_target_defaults(eyas_m6c_cases)

# Writes the M6d whole-aircraft data (VALIDATION.md; tools/plot_m6d.py).
add_executable(eyas_m6d_cases validation/m6d_cases.cpp)
target_include_directories(eyas_m6d_cases PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(eyas_m6d_cases PRIVATE eyas_internal)
target_compile_definitions(eyas_m6d_cases PRIVATE EYAS_VEHICLE_DATA_DIR="${PROJECT_SOURCE_DIR}/data")
eyas_target_defaults(eyas_m6d_cases)

add_executable(eyas_m7_cases validation/m7_cases.cpp)
target_include_directories(eyas_m7_cases PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(eyas_m7_cases PRIVATE eyas_internal)
target_compile_definitions(eyas_m7_cases PRIVATE EYAS_VEHICLE_DATA_DIR="${PROJECT_SOURCE_DIR}/data")
eyas_target_defaults(eyas_m7_cases)

add_executable(eyas_m7b_cases validation/m7b_cases.cpp)
target_include_directories(eyas_m7b_cases PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(eyas_m7b_cases PRIVATE eyas_internal)
target_compile_definitions(eyas_m7b_cases PRIVATE EYAS_VEHICLE_DATA_DIR="${PROJECT_SOURCE_DIR}/data")
eyas_target_defaults(eyas_m7b_cases)

add_executable(eyas_m7c_cases validation/m7c_cases.cpp)
target_include_directories(eyas_m7c_cases PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(eyas_m7c_cases PRIVATE eyas_internal)
target_compile_definitions(eyas_m7c_cases PRIVATE EYAS_VEHICLE_DATA_DIR="${PROJECT_SOURCE_DIR}/data")
eyas_target_defaults(eyas_m7c_cases)

# Writes Eyas's responses to Ballin's App. D and E inputs (D-046; tools/plot_ballin_app_d.py).
add_executable(eyas_ballin_app_d_cases validation/ballin_app_d_cases.cpp)
target_include_directories(eyas_ballin_app_d_cases PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(eyas_ballin_app_d_cases PRIVATE eyas_internal)
target_compile_definitions(eyas_ballin_app_d_cases PRIVATE EYAS_VEHICLE_DATA_DIR="${PROJECT_SOURCE_DIR}/data")
eyas_target_defaults(eyas_ballin_app_d_cases)
# Writes the M8d vortex-ring data (VALIDATION.md; tools/plot_m8d.py; D-049).
add_executable(eyas_m8d_cases validation/m8d_cases.cpp)
target_include_directories(eyas_m8d_cases PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(eyas_m8d_cases PRIVATE eyas_internal)
target_compile_definitions(eyas_m8d_cases PRIVATE EYAS_VEHICLE_DATA_DIR="${PROJECT_SOURCE_DIR}/data")
eyas_target_defaults(eyas_m8d_cases)

# Writes the M9 ground-effect data (VALIDATION.md; tools/plot_m9_ground_effect.py; D-050).
add_executable(eyas_m9_ground_effect_cases validation/m9_ground_effect_cases.cpp)
target_include_directories(eyas_m9_ground_effect_cases PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(eyas_m9_ground_effect_cases PRIVATE eyas_internal)
target_compile_definitions(eyas_m9_ground_effect_cases PRIVATE EYAS_VEHICLE_DATA_DIR="${PROJECT_SOURCE_DIR}/data")
eyas_target_defaults(eyas_m9_ground_effect_cases)

# Writes the M9 Stage B1′ data, Ballin's T700 (VALIDATION.md; tools/plot_m9_ballin.py; D-054).
add_executable(eyas_m9_ballin_engine_cases validation/m9_ballin_engine_cases.cpp)
target_include_directories(eyas_m9_ballin_engine_cases PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(eyas_m9_ballin_engine_cases PRIVATE eyas_internal)
target_compile_definitions(eyas_m9_ballin_engine_cases PRIVATE EYAS_VEHICLE_DATA_DIR="${PROJECT_SOURCE_DIR}/data")
eyas_target_defaults(eyas_m9_ballin_engine_cases)

# Writes the M9 Stage B engine data (VALIDATION.md; tools/plot_m9_engines.py; D-053).
add_executable(eyas_m9_engine_cases validation/m9_engine_cases.cpp)
target_include_directories(eyas_m9_engine_cases PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(eyas_m9_engine_cases PRIVATE eyas_internal)
target_compile_definitions(eyas_m9_engine_cases PRIVATE EYAS_VEHICLE_DATA_DIR="${PROJECT_SOURCE_DIR}/data")
eyas_target_defaults(eyas_m9_engine_cases)
add_executable(eyas_kaplita_cases validation/kaplita_cases.cpp)
target_include_directories(eyas_kaplita_cases PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(eyas_kaplita_cases PRIVATE eyas_internal)
target_compile_definitions(eyas_kaplita_cases PRIVATE EYAS_VEHICLE_DATA_DIR="${PROJECT_SOURCE_DIR}/data")
eyas_target_defaults(eyas_kaplita_cases)
add_executable(eyas_kaplita_transient_cases validation/kaplita_transient_cases.cpp)
target_include_directories(eyas_kaplita_transient_cases PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(eyas_kaplita_transient_cases PRIVATE eyas_internal)
target_compile_definitions(eyas_kaplita_transient_cases PRIVATE EYAS_VEHICLE_DATA_DIR="${PROJECT_SOURCE_DIR}/data")
eyas_target_defaults(eyas_kaplita_transient_cases)

add_executable(eyas_m7c_fletcher validation/m7c_fletcher.cpp)
target_include_directories(eyas_m7c_fletcher PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(eyas_m7c_fletcher PRIVATE eyas_internal)
target_compile_definitions(eyas_m7c_fletcher PRIVATE EYAS_VEHICLE_DATA_DIR="${PROJECT_SOURCE_DIR}/data")
eyas_target_defaults(eyas_m7c_fletcher)

# Writes the M8a long-run data (D-042; VALIDATION.md; tools/plot_m8a.py).
add_executable(eyas_m8a_cases validation/m8a_cases.cpp)
target_include_directories(eyas_m8a_cases PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(eyas_m8a_cases PRIVATE eyas_internal)
target_compile_definitions(eyas_m8a_cases PRIVATE EYAS_VEHICLE_DATA_DIR="${PROJECT_SOURCE_DIR}/data")
eyas_target_defaults(eyas_m8a_cases)

# Writes the M8c flight-control data (D-045; VALIDATION.md; tools/plot_m8c.py).
add_executable(eyas_m8c_cases validation/m8c_cases.cpp)
target_include_directories(eyas_m8c_cases PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(eyas_m8c_cases PRIVATE eyas_internal)
target_compile_definitions(eyas_m8c_cases PRIVATE EYAS_VEHICLE_DATA_DIR="${PROJECT_SOURCE_DIR}/data")
eyas_target_defaults(eyas_m8c_cases)
