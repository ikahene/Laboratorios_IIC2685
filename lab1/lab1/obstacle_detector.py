#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
import numpy as np


class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')
        self.bridge = CvBridge()

        # Parámetros ajustables
        self.declare_parameter('umbral_m', 0.5 + 0.1775)        # distancia máxima para considerar obstáculo [m]
        self.declare_parameter('ratio_nan', 0.10)      # fracción de NaN que implica obstáculo demasiado cerca
        self.declare_parameter('fila_ini', 0.25)       # banda vertical usada (fracción de la altura)
        self.declare_parameter('fila_fin', 0.75)
        self.umbral = self.get_parameter('umbral_m').value
        self.ratio_nan = self.get_parameter('ratio_nan').value
        self.fila_ini = self.get_parameter('fila_ini').value
        self.fila_fin = self.get_parameter('fila_fin').value

        self.estado_anterior = None

        self.subscription = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.image_callback,
            qos_profile_sensor_data)

        self.publisher = self.create_publisher(Vector3, '/occupancy_state', 10)

        self.get_logger().info('Detector iniciado. Esperando imágenes de la cámara...')

    def a_metros(self, depth):
        """Devuelve la imagen en metros como float32, con NaN donde no hay medición."""
        if depth.dtype == np.uint16:          # Kinect real: milímetros, 0 = sin medición
            d = depth.astype(np.float32) / 1000.0
            d[depth == 0] = np.nan
            return d
        d = depth.astype(np.float32)          # Simulador: metros
        d[d <= 0.0] = np.nan
        return d

    def region_ocupada(self, region):
        """Retorna (ocupada, distancia_minima) para una región de la imagen."""
        es_nan = np.isnan(region)
        frac_nan = es_nan.mean()
        validos = region[~es_nan]
        dist_min = float(validos.min()) if validos.size > 0 else float('inf')
        # Obstáculo si está dentro del umbral, o si el sensor "no ve" porque está demasiado cerca
        ocupada = (dist_min <= self.umbral) or (frac_nan >= self.ratio_nan)
        return ocupada, dist_min

    def image_callback(self, msg):
        self.get_logger().info('¡Conexión con la cámara establecida!', once=True)

        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            depth = self.a_metros(depth)

            alto, ancho = depth.shape
            banda = depth[int(alto * self.fila_ini):int(alto * self.fila_fin), :]

            tercio = ancho // 3
            izq = banda[:, :tercio]
            cen = banda[:, tercio:2 * tercio]
            der = banda[:, 2 * tercio:]

            ocu_i, d_i = self.region_ocupada(izq)
            ocu_c, d_c = self.region_ocupada(cen)
            ocu_d, d_d = self.region_ocupada(der)

            self.get_logger().debug(
                f'Distancias -> Izq: {d_i:.2f}, Cen: {d_c:.2f}, Der: {d_d:.2f}')

            salida = Vector3()
            salida.x = 1.0 if ocu_i else 0.0
            salida.y = 1.0 if ocu_c else 0.0
            salida.z = 1.0 if ocu_d else 0.0
            self.publisher.publish(salida)

            # Log solo cuando cambia el estado
            estado = (salida.x, salida.y, salida.z)
            if estado != self.estado_anterior:
                self.get_logger().info(f'Estado de ocupación: {estado}')
                self.estado_anterior = estado

        except Exception as e:
            self.get_logger().error(f'Error procesando imagen: {e}')


def main(args=None):
    rclpy.init(args=args)
    nodo = ObstacleDetector()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()