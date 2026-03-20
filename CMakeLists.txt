cmake_minimum_required(VERSION 3.16)
project(elevated_control LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_compile_options(-Wall -Wextra -Wpedantic)

# ---------- SOEM static library ----------
set(OS "linux")
set(OS_LIBS pthread rt)
file(GLOB OSAL_HEADERS
  ${CMAKE_CURRENT_SOURCE_DIR}/src/osal/osal.h
  ${CMAKE_CURRENT_SOURCE_DIR}/src/osal/${OS}/*.h)
file(GLOB OSAL_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/osal/${OS}/*.c)
file(GLOB OSHW_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/src/oshw/${OS}/*.h)
file(GLOB OSHW_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/oshw/${OS}/*.c)
file(GLOB SOEM_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/src/soem/*.h)
file(GLOB SOEM_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/soem/*.c)

add_library(elevated_soem STATIC
  ${SOEM_SOURCES}
  ${OSAL_SOURCES}
  ${OSHW_SOURCES}
)
set_property(TARGET elevated_soem PROPERTY POSITION_INDEPENDENT_CODE ON)
target_link_libraries(elevated_soem ${OS_LIBS})
target_include_directories(elevated_soem PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/osal>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/osal/${OS}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/oshw/${OS}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/soem>
)

# ---------- Main library ----------
find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(elevated_control SHARED
  src/arm_interface.cpp
  src/joint_admittance.cpp
  src/velocity_filter.cpp
  src/config_parsing.cpp
  src/dynamic_sim.cpp
)
target_include_directories(elevated_control PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/osal>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/osal/${OS}>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/oshw/${OS}>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/soem>
)
target_link_libraries(elevated_control
  PUBLIC spdlog::spdlog
  PRIVATE elevated_soem yaml-cpp pthread rt
)

# ---------- Examples ----------
add_executable(basic_usage examples/basic_usage.cpp)
target_link_libraries(basic_usage PRIVATE elevated_control)

set(EXAMPLE_CONFIG_YAMLS
  ${CMAKE_CURRENT_SOURCE_DIR}/examples/joint_limits.yaml
  ${CMAKE_CURRENT_SOURCE_DIR}/examples/elevate_config.yaml
)
foreach(yaml ${EXAMPLE_CONFIG_YAMLS})
  add_custom_command(
    TARGET basic_usage POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
      ${yaml}
      $<TARGET_FILE_DIR:basic_usage>
    VERBATIM
  )
endforeach()

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
  TARGETS elevated_control elevated_soem
  EXPORT elevated_control_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS basic_usage RUNTIME DESTINATION bin)
install(
  FILES examples/joint_limits.yaml examples/elevate_config.yaml
  DESTINATION share/elevated_control
)
