#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import termios
import tty
import select
import time


class Teleop(Node):
    def __init__(self):
        super().__init__("teleop")
        self.diccionario_comandos = {
            "i": (0.2,0),
            "j": (-0.2,0),
            "a": (0,1),
            "s": (0,-1),
            "q": (0.2,1),
            "w": (0.2,-1),
        }
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 16)

    def publicar_twist(self, linear, angular):
        mensaje = Twist()
        mensaje.linear.x = float(linear)
        mensaje.angular.z = float(angular)
        self.cmd_vel_pub.publish(mensaje)


def main(args=None):
    rclpy.init(args=args)
    node = Teleop()

    terminal = open("/dev/tty", "rb", buffering=0)
    fd = terminal.fileno()
    configuracion_original = termios.tcgetattr(fd)

    comando_actual = (0.0, 0.0)
    comando_pegado = False
    tiempo_ultima_tecla = 0.0

    print("==============================================================")
    print("  * i/j: Mover hacia delante / Mover hacia atrás")
    print("  * a/s: Girar hacia la izquierda / hacia la derecha")
    print("  * q/w: Curva hacia la izquierda / hacia la derecha")
    print("  * SHIFT + i/j/a/s/q/w: Comando Fijado (manos libres)")
    print("  * Espacio: Detener")
    print("==============================================================")

    try:
        tty.setcbreak(fd)

        while rclpy.ok():
            hay_tecla, _, _ = select.select([terminal], [], [], 0.05)

            if hay_tecla:
                tecla = terminal.read(1).decode(errors="ignore")

                if tecla == " ":
                    comando_actual = (0.0, 0.0)
                    comando_pegado = False
                elif tecla.lower() in node.diccionario_comandos:
                    comando_actual = node.diccionario_comandos[tecla.lower()]
                    comando_pegado = tecla.isupper()
                    tiempo_ultima_tecla = time.monotonic()

            if not comando_pegado and time.monotonic() - tiempo_ultima_tecla > 0.3:
                comando_actual = (0.0, 0.0)

            node.publicar_twist(*comando_actual)

    finally:
        if rclpy.ok():
            node.publicar_twist(0.0, 0.0)
        termios.tcsetattr(fd, termios.TCSADRAIN, configuracion_original)
        terminal.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
