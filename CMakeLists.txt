cmake_minimum_required(VERSION 3.16)
project(elevated_control LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_compile_options(-Wall -Wextra)

# ---------- SOEM static library ----------
add_subdirectory(external/SOEM)
set_target_properties(soem PROPERTIES POSITION_INDEPENDENT_CODE ON)

# ---------- Main library ----------
add_compile_options(-Wpedantic)

find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(elevated_control SHARED
  src/arm_interface.cpp
  src/joint_admittance.cpp
  src/velocity_filter.cpp
  src/config_parsing.cpp
  src/dynamic_sim.cpp
)
target_include_directories(elevated_control
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  PRIVATE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/external/SOEM>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/external/SOEM/soem>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/external/SOEM/osal>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/external/SOEM/osal/linux>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/external/SOEM/oshw/linux>
)
target_link_libraries(elevated_control
  PUBLIC spdlog::spdlog
  PRIVATE soem yaml-cpp pthread rt
)

# ---------- Examples ----------
add_executable(basic_usage examples/basic_usage.cpp)
target_link_libraries(basic_usage PRIVATE elevated_control)

# ---------- Tests ----------
option(BUILD_TESTING "Build tests" ON)
if(BUILD_TESTING)
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
    add_executable(${test_name} test/${test_name}.cpp)
    target_link_libraries(${test_name} PRIVATE elevated_control GTest::gtest GTest::gtest_main)
    gtest_discover_tests(${test_name})
  endforeach()
endif()

# ---------- Install ----------
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS elevated_control soem
  EXPORT elevated_control_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
