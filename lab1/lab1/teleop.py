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
        #Mapeamos las teclas con sus respectivos comandos (v, w)
        self.diccionario_comandos = {
            "i": (0.2,0), #Hacia delante, recto v = 0.2
            "j": (-0.2,0), #Hacia atrás, recto v = -0.2
            "a": (0,1), #Giro antihorario, w = 1
            "s": (0,-1), #Giro horario, w = -1
            "q": (0.2,1), #Giro antihorario + moverse adelante, v = 0.2 w= 1
            "w": (0.2,-1), #Giro horario + moverse adelante, v = 0.2 w= -1
        }
        #El nodo publica los comandos de velocidad Twist() al simulador
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 16)

    #Función auxiliar: Recibe las velocidades y las publica
    def publicar_twist(self, linear, angular):
        mensaje = Twist()
        mensaje.linear.x = float(linear)
        mensaje.angular.z = float(angular)
        self.cmd_vel_pub.publish(mensaje)


def main(args=None):
    #Iniciamos y creamos el nodo
    rclpy.init(args=args)
    node = Teleop()


    #Abrimos directamente la terminal (/dev/tty) en modo binario y sin buffer, para leer byte a byte.
    #Se usa /dev/tty y no sys.stdin porque con ros2 launch el stdin del nodo no es el teclado
    terminal = open("/dev/tty", "rb", buffering=0)
    #fd es el número (descriptor) con que el sistema operativo identifica esa terminal
    fd = terminal.fileno()
    #Guardamos la configuración actual de la terminal para restaurarla al salir
    configuracion_original = termios.tcgetattr(fd)

    #Comando (v, w) que se está publicando, parte detenido
    comando_actual = (0.0, 0.0)
    #True si el comando quedó fijo (tecla con SHIFT), así no se detiene al soltar la tecla
    comando_pegado = False
    #Instante en que se presionó la última tecla válida, para detectar cuando se suelta
    tiempo_ultima_tecla = 0.0

    #Instrucciones para el usuario
    print("==============================================================")
    print("  * i/j: Mover hacia delante / Mover hacia atrás")
    print("  * a/s: Girar hacia la izquierda / hacia la derecha")
    print("  * q/w: Curva hacia la izquierda / hacia la derecha")
    print("  * SHIFT + i/j/a/s/q/w: Comando Fijado (manos libres)")
    print("  * Espacio: Detener")
    print("==============================================================")

    #try/finally: pase lo que pase (incluso Ctrl+C) se ejecuta el bloque finally para dejar todo limpio
    try:
        #Modo cbreak: cada tecla llega apenas se presiona, sin esperar Enter y sin mostrarse en pantalla
        tty.setcbreak(fd)

        #Ciclo principal, corre hasta que se cierre ROS
        while rclpy.ok():
            #Esperamos máximo 0.05 s a que haya una tecla disponible; si no hay, igual seguimos.
            #Así el ciclo se repite unas 20 veces por segundo
            hay_tecla, _, _ = select.select([terminal], [], [], 0.05)

            if hay_tecla:
                #Leemos 1 byte y lo pasamos a texto (ignorando bytes raros, ej. flechas)
                tecla = terminal.read(1).decode(errors="ignore")

                #Espacio: detiene el robot y suelta un comando fijo
                if tecla == " ":
                    comando_actual = (0.0, 0.0)
                    comando_pegado = False
                #Tecla válida (en minúscula o mayúscula): buscamos su (v, w) en el diccionario
                elif tecla.lower() in node.diccionario_comandos:
                    comando_actual = node.diccionario_comandos[tecla.lower()]
                    #Si vino en mayúscula (con SHIFT) el comando queda fijo
                    comando_pegado = tecla.isupper()
                    tiempo_ultima_tecla = time.monotonic()

            #Si el comando no es fijo y pasaron más de 0.3 s sin teclas, asumimos que se soltó y frenamos.
            #Mientras se mantiene apretada, el sistema operativo repite la tecla y renueva este tiempo
            if not comando_pegado and time.monotonic() - tiempo_ultima_tecla > 0.3:
                comando_actual = (0.0, 0.0)

            #Publicamos siempre (aunque sea (0, 0)); el * separa la tupla en (linear, angular).
            #Publicar seguido evita que el simulador frene por no recibir comandos en 0.6 s
            node.publicar_twist(*comando_actual)

    #Al salir: frenamos el robot, devolvemos la terminal a como estaba y cerramos todo
    finally:
        if rclpy.ok(): #Si la conexión está andando
            node.publicar_twist(0.0, 0.0)
        termios.tcsetattr(fd, termios.TCSADRAIN, configuracion_original)
        terminal.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
