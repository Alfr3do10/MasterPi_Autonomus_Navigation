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
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
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
# CONFIGURACIÓN DE C++ (COMPONENTES)
# ==============================================================================

# 2. Componente C++ para la cámara
add_library(camera_node_component SHARED src/camera_component.cpp)
ament_target_dependencies(camera_node_component
  rclcpp
  rclcpp_components
  sensor_msgs
  image_transport
  cv_bridge
  OpenCV
  ament_index_cpp::ament_index_cpp
)

rclcpp_components_register_nodes(camera_node_component
  PLUGIN "masterpi_bringup::CameraComponent"
)

# 3. Componente C++ para detección de ArUco
add_library(aruco_detector_component SHARED src/aruco_detector_component.cpp)
ament_target_dependencies(aruco_detector_component
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  cv_bridge
  OpenCV
  ament_index_cpp::ament_index_cpp
)

rclcpp_components_register_nodes(aruco_detector_component
  PLUGIN "masterpi_bringup::ArucoDetectorComponent"
)

# 4. Instalar las librerías de componentes
install(TARGETS
  camera_node_component
  aruco_detector_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# ==============================================================================
# CONFIGURACIÓN DE PYTHON, LAUNCH Y CONFIGURACIONES
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
# Install Hiwonder SDK next to the Python executables.
install(
  DIRECTORY masterpi_bringup/hiwonder_sdk
  DESTINATION lib/${PROJECT_NAME}
)

# Install Hiwonder SDK inside the generated Python package too.
# Board.py imports:
#   from masterpi_bringup.hiwonder_sdk import yaml_handle
# so the SDK must also exist under Python site-packages/masterpi_bringup/.
install(
  DIRECTORY masterpi_bringup/hiwonder_sdk
  DESTINATION local/lib/python3.10/dist-packages/${PROJECT_NAME}
)
# ==============================================================================
# NODOS PYTHON EJECUTABLES
# ==============================================================================

install(
  PROGRAMS
    masterpi_bringup/line_follower_node.pyx
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
