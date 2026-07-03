cmake_minimum_required(VERSION 3.10)
project(cartesian_controller_simulation)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# We need C++17
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(cartesian_controller_base REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

# Mujoco comes as a pre-built library package.
# Extract this package somewhere and point CMake towards it during the build process.
#
# We auto-discover it so a plain `colcon build` works without any manual setup.
# Resolution order (first hit wins):
#   1. -DMUJOCO_DIR=... passed on the CMake/colcon command line
#   2. the MUJOCO_DIR environment variable
#   3. a versioned install under /opt (where the Docker image puts it)
#   4. a versioned install under $HOME
# The /opt and $HOME globs are sorted descending so the newest version wins.
if(NOT DEFINED MUJOCO_DIR)
        if(DEFINED ENV{MUJOCO_DIR})
                set(MUJOCO_DIR "$ENV{MUJOCO_DIR}")
        else()
                file(GLOB _mujoco_candidates
                        "/opt/mujoco-*"
                        "$ENV{HOME}/mujoco-*")
                list(SORT _mujoco_candidates ORDER DESCENDING)
                foreach(_candidate ${_mujoco_candidates})
                        if(EXISTS "${_candidate}/lib/libmujoco.so")
                                set(MUJOCO_DIR "${_candidate}")
                                break()
                        endif()
                endforeach()
        endif()
endif()

if(NOT MUJOCO_DIR OR NOT EXISTS "${MUJOCO_DIR}/lib/libmujoco.so")
        message(FATAL_ERROR
                "MuJoCo not found (searched -DMUJOCO_DIR, $ENV{MUJOCO_DIR}, "
                "/opt/mujoco-* and $ENV{HOME}/mujoco-*). Download the pre-built "
                "library package and point CMake to it via -DMUJOCO_DIR=<path> "
                "or the MUJOCO_DIR environment variable. See README.md.")
endif()
message(STATUS "Using MuJoCo at ${MUJOCO_DIR}")
set(MUJOCO_INCLUDE_DIRS ${MUJOCO_DIR}/include)
set(MUJOCO_LIBRARIES
        ${MUJOCO_DIR}/lib/libmujoco.so
)

# --------------------------------------------------------------------------------
# Make sure that library relocation works for both build and install.
# See here: https://gitlab.kitware.com/cmake/community/-/wikis/doc/cmake/RPATH-handling
# --------------------------------------------------------------------------------

set(CMAKE_SKIP_BUILD_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# The RPATH to use when installing, but only if it's not a system directory
list(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/lib" isSystemDir)
if("${isSystemDir}" STREQUAL "-1")
    set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
endif()

#--------------------------------------------------------------------------------
# Build
#--------------------------------------------------------------------------------
add_library(${PROJECT_NAME} SHARED
        src/system_interface.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
        cartesian_controller_base
        hardware_interface
        pluginlib
        rclcpp
)

target_link_libraries(${PROJECT_NAME}
        mujoco_simulator
)

pluginlib_export_plugin_description_file(hardware_interface cartesian_controller_simulation.xml)

#--------------------------------------------------------------------------------

add_library(mujoco_simulator SHARED
        src/mujoco_simulator.cpp
)

target_include_directories(mujoco_simulator PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${MUJOCO_INCLUDE_DIRS}
)

# X11 OpenGL: link with libmujoco210.so, libglew.so, libGL.so
target_link_libraries(mujoco_simulator PUBLIC
        ${MUJOCO_LIBRARIES}
)



#--------------------------------------------------------------------------------
# Install
#--------------------------------------------------------------------------------
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config etc launch urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME} mujoco_simulator
  DESTINATION lib
)


#--------------------------------------------------------------------------------
# Test
#--------------------------------------------------------------------------------
#if(BUILD_TESTING)
#  find_package(ament_lint_auto REQUIRED)
#  # the following line skips the linter which checks for copyrights
#  # uncomment the line when a copyright and license is not present in all source files
#  #set(ament_cmake_copyright_FOUND TRUE)
#  # the following line skips cpplint (only works in a git repo)
#  # uncomment the line when this package is not in a git repo
#  #set(ament_cmake_cpplint_FOUND TRUE)
#  ament_lint_auto_find_test_dependencies()
#endif()

ament_package()
