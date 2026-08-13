# Two kinds of tests registered here:
#
# 1. test_task_allocation -- real white-box unit test (Hungarian
#    algorithm, physical trajectory length), no OMPL dependency, compiles
#    standalone. Currently passing (12/12 checks).
#
# 2. smoke_test -- black-box integration test: builds nothing itself,
#    runs the real ugv_coordination binary end-to-end on the toy
#    instances and checks observable invariants (exit code, output files,
#    logged CSV rows, and the "same Gamma^0" cross-method guarantee).
#
# test_local_repair.cpp / test_splice.cpp / test_fallback.cpp /
# test_logging.cpp are intentionally NOT added here -- they are documented
# stubs (see each file's header comment) pending the LocalRepair
# concrete-type extraction noted in docs/REPRODUCIBILITY.md. Wiring an
# empty placeholder into CTest would report a false "PASS" that means
# nothing; better to be explicit that they don't exist yet.

add_executable(test_task_allocation test_task_allocation.cpp)
target_include_directories(test_task_allocation PRIVATE ${CMAKE_SOURCE_DIR}/include)
add_test(NAME test_task_allocation COMMAND test_task_allocation)

add_test(
  NAME smoke_test
  COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/smoke_test.py
          --binary $<TARGET_FILE:ugv_coordination>
          --instances-dir ${CMAKE_SOURCE_DIR}/experiments/full_pipeline/instances
          --vehicle-config ${CMAKE_SOURCE_DIR}/config/vehicle_config.yaml
)
set_tests_properties(smoke_test PROPERTIES TIMEOUT 180)
