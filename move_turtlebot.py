#!/usr/bin/env python3

import rclpy
import time
from numpy import pi
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist, PoseArray, Vector3
from tf_transformations import euler_from_quaternion


# Nodo principal
class Move_turtle(Node):

  def __init__(self):
    super().__init__('dead_reckoning_nav')

    # Grupo reentrante: permite que occupancy_cb corra mientras accion_mover_cb está bloqueado
    self.cb_group = ReentrantCallbackGroup()

    # Nodo publica velocidades
    self.cmd_vel_mux_pub = self.create_publisher(Twist, '/cmd_vel', 10)
    # Nodo escucha comandos
    self.poses_sub = self.create_subscription(
        PoseArray, 'goal_list', self.accion_mover_cb, 10,
        callback_group=self.cb_group)
    # Nodo escucha el estado de ocupación
    self.occ_sub = self.create_subscription(
        Vector3, '/occupancy_state', self.occupancy_cb, 10,
        callback_group=self.cb_group)

    # Estado del obstáculo
    self.obstaculo_presente = False
    self.estado_obstaculo = (0.0, 0.0, 0.0)

    # Guardamos la pose actual como la de inicio
    self.pose_actual = (1.0, 1.0, 0.0)  # x, y, yaw
    # Velocidades constantes
    self.vel_lin = 0.2  # [m/s]
    self.vel_rot = 1.0  # [rad/s]
    self.factor_giro = 1.111311640184305
    self.get_logger().info(f"Factor de giro en ejecución: {self.factor_giro}")

  # Recibe y aplica las velocidades al robot
  def aplicar_velocidad(self, speed_command_list: list):
    speed = Twist()

    for v, w, t in speed_command_list:
      speed.linear.x = float(v)
      speed.angular.z = float(w)

      restante = float(t)
      ultimo = time.monotonic()

      while rclpy.ok() and restante > 0.0:
        ahora = time.monotonic()
        dt = ahora - ultimo
        ultimo = ahora

        if self.obstaculo_presente:
          # Congelado: robot detenido y NO se descuenta tiempo del comando
          self.cmd_vel_mux_pub.publish(Twist())
        else:
          restante -= dt
          if restante > 0.0:
            self.cmd_vel_mux_pub.publish(speed)

        time.sleep(min(0.05, max(restante, 0.0)) if not self.obstaculo_presente else 0.05)

    # Detener el robot al terminar la lista.
    self.cmd_vel_mux_pub.publish(Twist())

  # Función auxiliar para crear los comandos con velocidad angular
  def comando_giro(self, yaw_desde, yaw_hacia):
    d = yaw_hacia - yaw_desde
    # Buscamos el giro mas corto, para ello comparamos con pi
    if d > pi:
      d = d - 2 * pi
    elif d < -pi:
      d = d + 2 * pi

    # Ignorar diferencias pequeñas producidas por 1.57 y 3.14.
    if abs(d) < 0.005:
      return (0.0, 0.0, 0.0)

    # Definimos hacia qué lado girar
    if d >= 0:
      w = self.vel_rot  # Antihorario
    else:
      w = -self.vel_rot
    return (0.0, w, abs(d) / self.vel_rot * self.factor_giro)

  # Función para determinar los comandos a aplicar
  def calcular_lista_velocidades(self, goal_pose):
    lista_comandos = []

    # Primero necesitamos conocer las distancias
    x_0, y_0, yaw_0 = self.pose_actual
    x_1, y_1, yaw_1 = goal_pose

    yaw_actual = yaw_0
    dis_horizontal = abs(x_1 - x_0)
    t_horizontal = dis_horizontal / self.vel_lin
    dis_vertical = abs(y_1 - y_0)
    t_vertical = dis_vertical / self.vel_lin

    # Primero nos movemos verticalmente
    if y_0 < y_1:
      lista_comandos.append(self.comando_giro(yaw_actual, pi / 2))
      yaw_actual = pi / 2
    elif y_0 > y_1:
      lista_comandos.append(self.comando_giro(yaw_actual, -pi / 2))
      yaw_actual = -pi / 2

    # Avanzamos
    if dis_vertical != 0.0:
      lista_comandos.append((self.vel_lin, 0.0, t_vertical))

    # Ahora nos movemos horizontalmente
    if x_0 < x_1:
      lista_comandos.append(self.comando_giro(yaw_actual, 0.0))
      yaw_actual = 0.0
    elif x_0 > x_1:
      lista_comandos.append(self.comando_giro(yaw_actual, pi))
      yaw_actual = pi

    # Avanzamos horizontalmente
    if dis_horizontal != 0.0:
      lista_comandos.append((self.vel_lin, 0.0, t_horizontal))

    # Dejamos el robot con su yaw final
    giro_final = self.comando_giro(yaw_actual, yaw_1)
    if giro_final[2] != 0:
      lista_comandos.append(giro_final)

    return lista_comandos

  def mover_robot_a_destino(self, goal_pose):
    speed_command_list = self.calcular_lista_velocidades(goal_pose)
    self.aplicar_velocidad(speed_command_list)

    # Asumimos que llegamos a destino
    self.pose_actual = goal_pose

    x, y, yaw = goal_pose
    self.get_logger().info(f'Llegué a la pose estimada: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}')

  def accion_mover_cb(self, msg):
    for goal_pose in msg.poses:
      x = goal_pose.position.x
      y = goal_pose.position.y
      quat = goal_pose.orientation

      # Pasamos el cuaternion a Euler
      roll, pitch, yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])

      self.mover_robot_a_destino((x, y, yaw))

  # Callback para actualizar el estado del obstáculo
  def occupancy_cb(self, msg):
    estado = (msg.x, msg.y, msg.z)
    self.obstaculo_presente = (msg.x == 1.0 or msg.y == 1.0 or msg.z == 1.0)

    # Loguear solo cuando cambia el estado
    if estado != self.estado_obstaculo:
      self.estado_obstaculo = estado
      if self.obstaculo_presente:
        if msg.x == 1.0:
          self.get_logger().info('obstacle left')
        if msg.y == 1.0:
          self.get_logger().info('obstacle center')
        if msg.z == 1.0:
          self.get_logger().info('obstacle right')
      else:
        self.get_logger().info('Camino libre, continuando movimiento')


def main(args=None):
  rclpy.init(args=args)

  nodo = Move_turtle()
  executor = MultiThreadedExecutor()
  executor.add_node(nodo)
  try:
    executor.spin()
  except KeyboardInterrupt:
    pass

  nodo.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()