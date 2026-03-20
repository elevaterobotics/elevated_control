find_package(GTest REQUIRED)
include(GoogleTest)
enable_testing()

foreach(test_name
  test_unit_conversions
  test_velocity_filter
  test_joint_admittance
  test_config_parsing
  test_joint_limits
  test_stop
)
  add_executable(${test_name} ${test_name}.cpp)
  target_link_libraries(${test_name} PRIVATE elevated_control GTest::gtest GTest::gtest_main)
  gtest_discover_tests(${test_name})
endforeach()
