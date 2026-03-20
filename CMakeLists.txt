cmake_minimum_required(VERSION 3.16)
project(elevated_control VERSION 0.0.0 LANGUAGES C CXX)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

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
add_compile_options(-Wpedantic)

find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/arm_interface.cpp
  src/joint_admittance.cpp
  src/velocity_filter.cpp
  src/config_parsing.cpp
  src/dynamic_sim.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}
  PUBLIC spdlog::spdlog
  PRIVATE elevated_soem yaml-cpp pthread rt
)

# ---------- Examples ----------
option(BUILD_EXAMPLES "Build examples" ON)
if(BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()

# ---------- Tests ----------
option(BUILD_TESTING "Build tests" ON)
if(BUILD_TESTING)
  add_subdirectory(test)
endif()

# ---------- Install ----------
install(
  DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(
  TARGETS elevated_soem
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
