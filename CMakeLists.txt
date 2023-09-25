# Update CMakeLists.txt from odri_control_interface to adapt it to Bolt

#
# Copyright (c) 2020, New York University and Max Planck Gesellschaft.
#
# License BSD-3 clause
#

cmake_minimum_required(VERSION 3.14)

# ----------------------------------------------------
# --- CXX FLAGS --------------------------------------
# ----------------------------------------------------

set(CXX_DISABLE_WERROR True)
set(CMAKE_VERBOSE_MAKEFILE True)

# These variables have to be defined before running SETUP_PROJECT
set(PROJECT_NAME ros2_hardware_interface_odri)
set(PROJECT_DESCRIPTION
    "Common interface for controlling robots build with the odri master board.")
set(PROJECT_URL "https://github.com/stack-of-tasks/${PROJECT_NAME}")
set(PROJECT_USE_CMAKE_EXPORT TRUE)

# Check if the submodule cmake have been initialized
set(JRL_CMAKE_MODULES "${CMAKE_CURRENT_LIST_DIR}/cmake")
if(NOT EXISTS "${CMAKE_SOURCE_DIR}/cmake/base.cmake")
  message(STATUS "JRL cmakemodules not found. Let's fetch it.")
  include(FetchContent)
  FetchContent_Declare(
    "jrl-cmakemodules"
    GIT_REPOSITORY "https://github.com/jrl-umi3218/jrl-cmakemodules.git")
  FetchContent_MakeAvailable("jrl-cmakemodules")
  FetchContent_GetProperties("jrl-cmakemodules" SOURCE_DIR JRL_CMAKE_MODULES)
endif()

# --- OPTIONS ----------------------------------------
option(BUILD_PYTHON_INTERFACE "Build the python binding" ON)
option(PYTHON_STANDARD_LAYOUT "Enable standard Python package layout" ON)
option(PYTHON_DEB_LAYOUT "Enable Debian-style Python package layout" OFF)

include("${JRL_CMAKE_MODULES}/base.cmake")
include("${JRL_CMAKE_MODULES}/boost.cmake")
include("${JRL_CMAKE_MODULES}/python.cmake")
include("${JRL_CMAKE_MODULES}/ide.cmake")

compute_project_args(PROJECT_ARGS LANGUAGES CXX)
project(${PROJECT_NAME} ${PROJECT_ARGS})
check_minimal_cxx_standard(11 ENFORCE)

# ----------------------------------------------------
# --- DEPENDENCIES -----------------------------------
# ----------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(odri_control_interface REQUIRED)
find_package(controller_interface REQUIRED)
find_package(master_board_sdk REQUIRED)

add_project_dependency(yaml-cpp CONFIG REQUIRED)
add_project_dependency(Eigen3 REQUIRED)
add_project_dependency(master_board_sdk REQUIRED)
add_project_dependency(odri_control_interface REQUIRED)
add_project_dependency(controller_interface REQUIRED)

# Set component to fetch from boost Get the python interface for the bindings
if(BUILD_PYTHON_INTERFACE)
  add_project_dependency(eigenpy 3.0.0 REQUIRED)
endif(BUILD_PYTHON_INTERFACE)

#ament_target_dependencies(rclcpp)


pluginlib_export_plugin_description_file(hardware_interface "${PROJECT_NAME}.xml")

ament_export_dependencies(odri_control_interface)




# ----------------------------------------------------
# --- INCLUDE ----------------------------------------
# ----------------------------------------------------

# --- MAIN LIBRARY -------------------------------------------------------------
set(${PROJECT_NAME}_SOURCES src/system_odri.cpp)
set(${PROJECT_NAME}_HEADERS
    "include/${PROJECT_NAME}/system_interface_odri.hpp"
    "include/${PROJECT_NAME}/system_odri.hpp"
    "include/${PROJECT_NAME}/visibility_control.h")

add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SOURCES}
    ${${PROJECT_NAME}_HEADERS})
target_link_libraries(${PROJECT_NAME} ${YAML_CPP_LIBRARIES}
 master_board_sdk::master_board_sdk
 Eigen3::Eigen
 rclcpp::rclcpp
 odri_control_interface::odri_control_interface)

ament_target_dependencies(
  ${PROJECT_NAME}
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
)

target_include_directories(
  ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                         $<INSTALL_INTERFACE:include>)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${TARGETS_EXPORT_NAME}
  DESTINATION lib)

# --- BINDINGS -----------------------------------------------------------------

if(BUILD_PYTHON_INTERFACE)

  # --- Build the wrapper library --- #
  set(${PYWRAP}_INSTALL_DIR ${PYTHON_SITELIB})
  target_link_libraries(${PYWRAP} ${PROJECT_NAME} eigenpy::eigenpy)
  get_relative_rpath(${${PYWRAP}_INSTALL_DIR} ${PYWRAP}_INSTALL_RPATH)
  set_target_properties(${PYWRAP} PROPERTIES INSTALL_RPATH
                                             "${${PYWRAP}_INSTALL_RPATH}")
  install(TARGETS ${PYWRAP} DESTINATION ${${PYWRAP}_INSTALL_DIR})

  # --- Allow to do: make python --- #
  add_custom_target(python)
  set_target_properties(python PROPERTIES EXCLUDE_FROM_DEFAULT_BUILD True)

endif(BUILD_PYTHON_INTERFACE)



ament_package()
