cmake_minimum_required(VERSION 3.16)
project(oculus_ros2 VERSION 1.0)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -ggdb3)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(oculus_interfaces REQUIRED)

# oculus_driver is an external CMake-only library (not a colcon package);
# located via CMAKE_PREFIX_PATH, not find_package's normal ament search.
find_package(oculus_driver REQUIRED)

# OpenCV is used directly for the fan-image render in sonar_viewer.cpp. Deliberately
# UNVERSIONED: the Jetson's only OpenCV with dev files is NVIDIA's 4.8.0, and the old
# `find_package(OpenCV 4.5.4 REQUIRED)` silently accepted it as ">= 4.5.4" while cv_bridge
# still dragged in the 4.5.4 ABI -> two OpenCVs in one binary. cv_bridge is intentionally
# NOT a dependency any more; sonar_viewer.cpp fills sensor_msgs::msg::Image by hand instead.
# Note also: do not use pkg-config for OpenCV on the Jetson -- its opencv4.pc is stale and
# points at a /usr/local prefix that no longer exists. find_package resolves correctly.
find_package(OpenCV REQUIRED)

# oculus_driver's shared lib lives outside the ROS install tree, so nothing sources its
# path at runtime. Bake it into the executable's RPATH at build time instead of relying on
# LD_LIBRARY_PATH (which needs setting on every machine/launch). oculus_driver_DIR is set by
# find_package() above to <prefix>/lib/cmake/oculus_driver, so this stays correct regardless
# of which CMAKE_PREFIX_PATH was used to build.
get_filename_component(OCULUS_DRIVER_LIB_DIR "${oculus_driver_DIR}/../.." ABSOLUTE)

add_executable(oculus_sonar_node
    src/oculus_sonar_node.cpp
    src/sonar_viewer.cpp
)
target_include_directories(oculus_sonar_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(oculus_sonar_node PRIVATE
    oculus_driver
)
set_target_properties(oculus_sonar_node PROPERTIES
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH "${OCULUS_DRIVER_LIB_DIR}"
)
ament_target_dependencies(oculus_sonar_node PUBLIC
    rclcpp
    oculus_interfaces
    rcl_interfaces
    sensor_msgs
    OpenCV
)

install(PROGRAMS scripts/display_oculus_file.py scripts/oculus_to_rosbag.py DESTINATION bin)
install(DIRECTORY launch cfg DESTINATION share/${PROJECT_NAME})
install(TARGETS oculus_sonar_node DESTINATION lib/${PROJECT_NAME})

ament_package()
