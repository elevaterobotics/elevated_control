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
  test_spring_adjust
  test_dial_normalization
  test_deadman_mode_change
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

# config_verification helpers test (no EtherCAT / library dependency)
add_executable(test_config_verification test_config_verification.cpp
  $<TARGET_OBJECTS:config_verification_helpers>)
target_link_libraries(test_config_verification PRIVATE GTest::gtest GTest::gtest_main)
target_include_directories(test_config_verification PRIVATE
  ${PROJECT_SOURCE_DIR}/examples)
target_compile_definitions(test_config_verification PRIVATE
  TEST_RESOURCES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/resources")
gtest_discover_tests(test_config_verification
  DISCOVERY_MODE PRE_TEST)
