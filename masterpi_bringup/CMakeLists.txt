cmake_minimum_required(VERSION 3.8)
project(masterpi_bringup)

# Buscar estándares de C++
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. BUSCAR DEPENDENCIAS DE ROS 2 Y SISTEMA
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

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

# 5. Instalar carpetas de recursos, configuraciones y archivos launch
install(
  DIRECTORY config launch actions resource
  DESTINATION share/${PROJECT_NAME}
)

# 6. Instalar tus scripts de Python existentes
install(
  PROGRAMS
  masterpi_bringup/__init__.py
  masterpi_bringup/camera_node.py
  masterpi_bringup/line_follower_node.py
  masterpi_bringup/line_follower_pose_node.py
  masterpi_bringup/arm_motion_node.py
  masterpi_bringup/aruco.py
  masterpi_bringup/battery_node.py
  masterpi_bringup/motor_node.py
  masterpi_bringup/odom_node.py
  masterpi_bringup/servo_node.py
  masterpi_bringup/sonar_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# ==============================================================================
# CIERRE DE PAQUETE
# ==============================================================================
ament_python_install_package(${PROJECT_NAME})
ament_package()
