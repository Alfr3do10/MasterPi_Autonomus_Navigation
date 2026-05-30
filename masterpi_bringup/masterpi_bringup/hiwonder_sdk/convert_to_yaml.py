#!/usr/bin/env python3
import os
import numpy as np
import cv2
# Importamos la herramienta de ROS 2 para buscar paquetes
from ament_index_python.packages import get_package_share_directory

# 1. ROS 2 encuentra automáticamente la ruta base de instalación de tu paquete
package_path = get_package_share_directory('masterpi_bringup')

# 2. Construimos la ruta uniendo las piezas de forma segura
# (Nota: En paquetes instalados, la carpeta suele ser 'config/calibration/...')
npz_path = os.path.join(package_path, 'config', 'calibration', 'calibration_param.npz')
yaml_path = os.path.join(package_path, 'config', 'calibration', 'calibration_param.yaml')

print(f"Buscando archivo en ruta dinámica de ROS 2:\n{npz_path}\n")

try:
    data = np.load(npz_path)
    mtx = data['mtx_array']
    dist = data['dist_array']
    
    fs = cv2.FileStorage(yaml_path, cv2.FileStorage_WRITE)
    fs.write("camera_matrix", mtx)
    fs.write("distortion_coefficients", dist)
    fs.release()
    print(f"[ÉXITO] Archivo YAML generado en:\n{yaml_path}")

except Exception as e:
    print(f"[ERROR] No se pudo realizar la conversión automática: {e}")
    print("\n💡 Consejo: Asegúrate de haber hecho 'colcon build' y 'source install/setup.bash' para que ROS 2 encuentre el paquete.")