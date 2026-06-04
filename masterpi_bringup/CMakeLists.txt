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
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)# OpenCV nativo para C++

# ==============================================================================
# CONFIGURACIÓN DE C++ (NODO DE LA CÁMARA)
# ==============================================================================

# 2. Definir el ejecutable de C++ y su archivo fuente
add_executable(camera_node_cpp src/camera_node.cpp)

# 3. Vincular las librerías necesarias al ejecutable
target_link_libraries(camera_node_cpp
  PRIVATE
  rclcpp::rclcpp
  cv_bridge::cv_bridge
  image_transport::image_transport
  ${sensor_msgs_LIBRARIES}
  ${OpenCV_LIBRARIES}
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

target_link_libraries(aruco_detector_component
  rclcpp::rclcpp
  cv_bridge::cv_bridge
  ${OpenCV_LIBRARIES}
)
rclcpp_components_register_node(TARGET aruco_detector_component
  PLUGIN "masterpi_bringup::ArucoDetectorComponent"
)

# 4. Instalar los binarios y la librería de componente
install(TARGETS
  camera_node_cpp
  aruco_detector_component
  DESTINATION lib/${PROJECT_NAME}
)

# ==============================================================================
# CONFIGURACIÓN DE PYTHON, LAUNCH Y CONFIGURACIONES (MANTENER LO EXISTENTE)
# ==============================================================================

# 5. Instalar carpetas de recursos, configuraciones y archivos launch
install(
  DIRECTORY config launch actions resource
  DESTINATION share/${PROJECT_NAME}
)

# 6. Instalar tus scripts de Python existentes para que ROS 2 los pueda ejecutar
# (Asumiendo que tus scripts ejecutables de Python están dentro de 'masterpi_bringup/')
install(
  PROGRAMS
  masterpi_bringup/__init__.py
  masterpi_bringup/camera_node.py
  masterpi_bringup/line_follower_node.py
  masterpi_bringup/aruco.py
  masterpi_bringup/battery_node.py
  masterpi_bringup/motor_node.py
  masterpi_bringup/odom_node.py
  masterpi_bringup/servo_node.py
  masterpi_bringup/sonar_node.py
  # Añade aquí abajo los nombres de los otros scripts .py que quieras ejecutar como nodos
  # Ejemplo: masterpi_bringup/otro_nodo.py
  DESTINATION lib/${PROJECT_NAME}
)

# ==============================================================================
# CIERRE DE PAQUETE
# ==============================================================================
ament_package()