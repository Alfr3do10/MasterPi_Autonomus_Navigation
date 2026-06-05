cmake_minimum_required(VERSION 3.8)
project(masterpi_bringup)

# ==============================================================================
# CONFIGURACIÓN GENERAL
# ==============================================================================

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ==============================================================================
# DEPENDENCIAS
# ==============================================================================

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

find_package(rosidl_default_generators REQUIRED)

# ==============================================================================
# INTERFACES ROS 2
# ==============================================================================

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/RunArmMotion.srv"
)

# ==============================================================================
# NODO C++ DE CÁMARA
# ==============================================================================

add_executable(camera_node_cpp src/camera_node.cpp)

target_link_libraries(camera_node_cpp
  PRIVATE
  rclcpp::rclcpp
  cv_bridge::cv_bridge
  image_transport::image_transport
  ${sensor_msgs_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

install(TARGETS
  camera_node_cpp
  DESTINATION lib/${PROJECT_NAME}
)

# ==============================================================================
# ARCHIVOS DE CONFIGURACIÓN, LAUNCH Y RECURSOS
# ==============================================================================

install(
  DIRECTORY
    config
    launch
    actions
    resource
  DESTINATION share/${PROJECT_NAME}
)

# ==============================================================================
# SDK HIWONDER
# ==============================================================================
# Lo instalamos junto a los ejecutables Python en lib/masterpi_bringup.
# Así los nodos pueden usar:
#   from hiwonder_sdk import Board, yaml_handle
# sin interferir con el paquete generado masterpi_bringup.srv.

install(
  DIRECTORY masterpi_bringup/hiwonder_sdk
  DESTINATION lib/${PROJECT_NAME}
)

# ==============================================================================
# NODOS PYTHON EJECUTABLES
# ==============================================================================

install(
  PROGRAMS
    masterpi_bringup/line_follower_node.py
    masterpi_bringup/line_follower_pose_node.py
    masterpi_bringup/arm_motion_node.py
    masterpi_bringup/arm_motion_service_node.py
    masterpi_bringup/mission_manager_node.py
    masterpi_bringup/station_maneuver_node.py
    masterpi_bringup/aruco.py
    masterpi_bringup/battery_node.py
    masterpi_bringup/motor_node.py
    masterpi_bringup/odom_node.py
    masterpi_bringup/servo_node.py
    masterpi_bringup/sonar_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# ==============================================================================
# EXPORTS
# ==============================================================================

ament_export_dependencies(rosidl_default_runtime)

ament_package()
