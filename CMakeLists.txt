cmake_minimum_required(VERSION 3.16)
project(elevated_control VERSION 0.0.0 LANGUAGES C CXX)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

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
    $<INSTALL_INTERFACE:include>
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
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
install(
  TARGETS elevated_control
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(
  TARGETS soem
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# ---------- Exporting for CMake find_package ----------
set(${PROJECT_NAME}_INSTALL_CMAKEDIR "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}")

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)

configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  INSTALL_DESTINATION "${${PROJECT_NAME}_INSTALL_CMAKEDIR}"
)

install(
  EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION "${${PROJECT_NAME}_INSTALL_CMAKEDIR}"
)

install(
  FILES
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  DESTINATION "${${PROJECT_NAME}_INSTALL_CMAKEDIR}"
)
