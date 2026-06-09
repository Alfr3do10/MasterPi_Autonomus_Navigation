# masterpi_bringup

## Descripción General
Este paquete de ROS 2 proporciona la capa de control principal y las rutinas de autonomía para un robot móvil tipo MasterPi. El sistema está diseñado para ejecutar misiones complejas de manera autónoma, integrando navegación por seguimiento de línea, detección espacial mediante marcadores ArUco, rutinas de seguridad reactivas con sensores ultrasónicos y manipulación de objetos mediante un brazo robótico integrado.

## Características Principales

* **Seguidor de Línea Robusto:** Implementación de procesamiento de imágenes con OpenCV que soporta modos de detección por brillo o HSV. Incluye un controlador PID para ajustes de trayectoria, filtrado de centro de línea y máscaras de Región de Interés (ROI) dinámicas.
* **Gestión de Misiones con ArUco:** Uso de `ArucoDetectorComponent` para la detección de estaciones de trabajo. El robot se aproxima, alinea y ejecuta acciones de *pickup* o *drop* dependiendo del ID del marcador detectado y la lógica de la misión.
* **Seguridad Ultrasónica Integrada:** Un nodo dedicado al sonar monitorea el frente del robot de forma independiente, deteniendo el movimiento (`/cmd_vel`) y deshabilitando el seguidor de línea ante obstáculos inminentes, con lógica de histéresis para evitar paradas en falso.
* **Control Suavizado del Brazo Robótico:** Interfaz con el hardware del brazo (Hiwonder SDK) que permite la definición de poses estáticas y secuencias complejas. Incorpora interpolación suave (Ease-in/Ease-out) y calibración de desviación por servo para mayor precisión.

---

## Arquitectura de Nodos

La arquitectura del paquete se divide en los siguientes componentes principales:

| Nodo | Archivo | Descripción Funcional |
| :--- | :--- | :--- |
| **`mission_manager_node`** | `mission_manager_node.py` | Máquina de estados principal. Orquesta la misión completa, habilitando/deshabilitando la navegación, evaluando las detecciones de ArUco y solicitando movimientos del brazo. |
| **`line_follower_node`** | `line_follower_node.py` | Procesa `/camera/image_raw` y publica comandos en `/cmd_vel`. Utiliza un controlador PID para mantener el robot sobre la línea basándose en el cálculo de momentos de la imagen. |
| **`sonar_safety_node`** | `sonar_safety_node.py` | Interrumpe la navegación si la lectura en `/sonar/range` cae por debajo del umbral crítico de seguridad. Gestiona la reanudación automática tras despejarse la ruta. |
| **`arm_motion_node`** | `arm_motion_node.py` | Ejecuta perfiles de movimiento físicos en los servomotores. Soporta modo de simulación (`mock_hardware`) y lee compensaciones geométricas (`deviation`). |
| **`camera_node`** | `CameraComponent` (C++) | Nodo en C++ de alto rendimiento para la publicación del flujo de video RAW bajo el esquema de Intra-Process Communication. |
| **`aruco_detector`** | `ArucoDetectorComponent` | Detecta marcadores ArUco en el flujo de video y calcula el error de `yaw` y distancia en metros para posicionamiento preciso. |

---

## Archivos de Lanzamiento (Launch Files)

| Launch File | Propósito Principal |
| :--- | :--- |
| **`mission_manager.launch.py`** | El archivo principal para misiones autónomas. Arranca la pila completa del robot mediante temporizadores (`TimerAction`) y soporta múltiples argumentos para pruebas modulares (`use_mock_hardware`, `start_sonar`, etc.). |
| **`line_follower.launch.py`** | Entorno de despliegue enfocado puramente en la navegación de línea. Arranca la cámara, posiciona el brazo en modo transporte y habilita la locomoción de la base. |
| **`aruco_detector_container.launch.py`** | Aislado para pruebas de percepción visual. Arranca un `ComposableNodeContainer` para procesar el flujo de video y buscar marcadores ArUco con cero copias de memoria (Intra-Process). |

---

## Parámetros de Configuración

El comportamiento del robot es altamente configurable mediante los archivos YAML ubicados en el directorio `config/`:

* **`line_follower_params.yaml`:** Ajusta las ganancias del PID (`kp_angular`, `kd_angular`), establece límites de velocidad lineal/angular, áreas de los contornos para OpenCV y coordina las posiciones de las ROIs en el encuadre (típicamente 320x240).
* **`mission_manager_params.yaml`:**
  Define los disparadores de misión. Configura a qué distancia (metros) iniciar las secuencias de ArUco, los IDs válidos para interactuar, tiempos de *cooldown* entre estaciones y el control proporcional para aproximación/alineación.
* **`sonar_safety_params.yaml`:**
  Configura la lógica de seguridad, definiendo `stop_distance_m` (detención de emergencia) y `clear_distance_m` (distancia requerida para reanudar operaciones).
* **`arm_motion_params.yaml` (y variables locales):**
  Almacena las secuencias de poses (Ej. `home`, `pickup`, `drop`, `carry_line_follower`), el tiempo base de los motores y el identificador de los servos clave (como el *gripper*).

---

## Requisitos y Dependencias

* Sistema Operativo: Ubuntu (basado en Debian)
* ROS 2 (Distribución correspondiente)
* Bibliotecas de Python: `numpy`, `opencv-python` (`cv2`), `cv_bridge`
* Paquetes de ROS 2: `rclpy`, `launch_ros`, `geometry_msgs`, `sensor_msgs`, `std_msgs`
* Hardware Específico: Hiwonder MasterPi (se requiere la importación de `hiwonder_sdk` para la actuación en hardware real).
