# Real-time timing benchmark (spec §7 rule 5, D-005). Uses the internal
# executive to time a single step(), so it links eyas_internal.
find_package(Threads REQUIRED)

add_executable(eyas_bench eyas_bench.cpp)
target_link_libraries(eyas_bench PRIVATE eyas_internal Threads::Threads)
target_compile_definitions(eyas_bench PRIVATE
    EYAS_BENCH_DEFAULT_CONFIG="${PROJECT_SOURCE_DIR}/data/uh60a/uh60a.yaml")
eyas_target_defaults(eyas_bench)

if(EYAS_BUILD_TESTS)
    # Smoke test only: runs a few frames and checks it completes. Never a
    # timing gate (CI machines are shared); the exit code 3 (budget not met)
    # is accepted.
    add_test(NAME Bench.Smoke COMMAND eyas_bench --frames 30)
    set_tests_properties(Bench.Smoke PROPERTIES SKIP_RETURN_CODE 3)
endif()
