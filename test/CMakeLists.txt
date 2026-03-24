find_package(GTest REQUIRED)
include(GoogleTest)

foreach(test_name
  test_unit_conversions
  test_velocity_filter
  test_joint_admittance
  test_config_parsing
  test_joint_limits
  test_stop
  test_state_machine
  test_streaming_spring_slot
)
  add_executable(${test_name} ${test_name}.cpp)
  target_link_libraries(${test_name} PRIVATE ${PROJECT_NAME} GTest::gtest GTest::gtest_main)
  # PRE_TEST: do not run the binary at link time (avoids gtest discovery loading an
  # older libelevated_control.so from LD_LIBRARY_PATH before the freshly built one).
  # ENVIRONMENT: when ctest runs discovery/tests, prefer ${CMAKE_BINARY_DIR} on LD_LIBRARY_PATH.
  gtest_discover_tests(${test_name}
    DISCOVERY_MODE PRE_TEST
    PROPERTIES
      ENVIRONMENT "LD_LIBRARY_PATH=${CMAKE_BINARY_DIR}:$ENV{LD_LIBRARY_PATH}"
  )
endforeach()
