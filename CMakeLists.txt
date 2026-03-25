cmake_minimum_required(VERSION 3.18)
project(elevated_control VERSION 0.0.0 LANGUAGES C CXX)

include(CTest)
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# Help the loader find libelevated_control.so in this build tree when RUNPATH is used.
set(CMAKE_BUILD_RPATH "${CMAKE_BINARY_DIR}")

add_compile_options(-Wall -Wextra)
# SOEM is C and uses -Werror; -Wpedantic on C breaks the submodule build. Apply to C++ only.
add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-Wpedantic>)

# ---------- SOEM static library ----------
add_subdirectory(external/SOEM)
set_target_properties(soem PROPERTIES POSITION_INDEPENDENT_CODE ON)

# ---------- Main library ----------

find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/arm_interface.cpp
  src/joint_admittance.cpp
  src/spring_adjust.cpp
  src/velocity_filter.cpp
  src/config_parsing.cpp
  src/dynamic_sim.cpp
)
target_include_directories(${PROJECT_NAME}
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
target_link_libraries(${PROJECT_NAME}
  PUBLIC spdlog::spdlog
  PRIVATE soem yaml-cpp pthread rt
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
