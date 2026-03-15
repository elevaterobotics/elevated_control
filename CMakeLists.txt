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
