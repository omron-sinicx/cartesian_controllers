cmake_minimum_required(VERSION 3.5)
project(cartesian_compliance_controller)

set(CMAKE_CXX_STANDARD 17)
set(ADDITIONAL_COMPILE_OPTIONS -Wall -Wextra -Wpedantic -Wno-unused-parameter)
add_compile_options(${ADDITIONAL_COMPILE_OPTIONS})

# Set build type to "Release" by default
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
add_definitions(-DEIGEN_MPL2_ONLY)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cartesian_controller_base REQUIRED)
find_package(cartesian_motion_controller REQUIRED)
find_package(cartesian_force_controller REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetStiffness.srv"
)

# Convenience variable for dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
        rclcpp
        cartesian_controller_base
        cartesian_motion_controller
        cartesian_force_controller
        Eigen3
)

ament_export_dependencies(
        ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

#--------------------------------------------------------------------------------
# Libraries
#--------------------------------------------------------------------------------
add_library(${PROJECT_NAME}_plugin SHARED
  src/cartesian_compliance_controller.cpp
)

set_target_properties(${PROJECT_NAME}_plugin PROPERTIES OUTPUT_NAME ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME}_plugin
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(${PROJECT_NAME}_plugin
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}_plugin "${cpp_typesupport_target}")

# Prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME}_plugin PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

#--------------------------------------------------------------------------------
# Install and export
#--------------------------------------------------------------------------------

pluginlib_export_plugin_description_file(controller_interface cartesian_compliance_controller_plugin.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}_plugin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}_plugin
)

ament_package()
