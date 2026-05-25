#!/usr/bin/env python3

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')

        # 1. Parámetros del nodo
        self.declare_parameter('aruco_dictionary', 'DICT_4X4_50')
        self.declare_parameter('show_image', False)

        self.show_image = self.get_parameter('show_image').value
        dictionary_name = self.get_parameter('aruco_dictionary').value

        # 2. Configuración específica de ArUco para OpenCV 4.5.4
        # En esta versión se accede a los diccionarios mediante cv2.aruco.Dictionary_get()
        try:
            dictionary_id = getattr(cv2.aruco, dictionary_name)
            self.aruco_dictionary = cv2.aruco.Dictionary_get(dictionary_id)
            self.aruco_parameters = cv2.aruco.DetectorParameters_create()
        except AttributeError:
            self.get_logger().error(f'El diccionario "{dictionary_name}" no existe en cv2.aruco.')
            # Diccionario de respaldo por si acaso
            self.aruco_dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
            self.aruco_parameters = cv2.aruco.DetectorParameters_create()

        self.bridge = CvBridge()

        # 3. Suscriptor al tópico de tu cámara
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.get_logger().info('Nodo Detector de ArUco (v4.5.4) iniciado.')
        self.get_logger().info(f'Buscando marcadores del diccionario: {dictionary_name}')

    def image_callback(self, msg):
        try:
            # 'passthrough' hace que si viene en gris se quede en gris, 
            # y si viene a color se quede a color automáticamente.
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'Error al convertir imagen: {str(e)}')
            return

        # 4. Detección de marcadores con la API de OpenCV 4.5.4
        corners, ids, rejected = cv2.aruco.detectMarkers(
            frame, 
            self.aruco_dictionary, 
            parameters=self.aruco_parameters
        )

        # 5. Procesar e imprimir los IDs encontrados
        if ids is not None:
            # Limpiamos la matriz de IDs para tener una lista de enteros fácil de leer
            detected_ids = [int(id_array[0]) for id_array in ids]
            self.get_logger().info(f'¡ArUco detectado! IDs: {detected_ids}')

            # Dibujar los recuadros verdes y el ID sobre el objeto si el visor está activo
            if self.show_image:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        else:
            if self.show_image:
                cv2.putText(frame, 'Buscando ArUco...', (20, 40), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        # 6. Mostrar la ventana con el feedback visual
        if self.show_image:
            cv2.imshow("ArUco Detector - OpenCV 4.5.4", frame)
            cv2.waitKey(1)

    def destroy_node(self):
        if self.show_image:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
