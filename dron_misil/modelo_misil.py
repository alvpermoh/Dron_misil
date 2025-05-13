#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool
import numpy as np

class MisilModel(Node):
    def __init__(self):
        super().__init__('misil_model_node')

        self.dt = 0.1  # intervalo de tiempo
        self.max_speed = 10.0  # velocidad máxima
        self.Tam = 300.0  # tamaño del entorno

        self.misil_pos = np.random.uniform(0, self.Tam, size=(3,))

        self.missile_speed = 30  # Velocidad del misil (ajustable)
        self.missile_turn_rate = 0.1  # Tasa de giro del misil (ajustable)

        self.missile_dir = np.array([1.0, 0.0, 0.0])  # Dirección inicial (por ejemplo hacia X)

        self.dron_pos = np.random.uniform(0, self.Tam, size=(3,))  # Inicializar alguna posición para evitar errores

        # Suscriptores
        self.reset_sub = self.create_subscription(
            Bool, '/dron/reset', self.reset_callback, 10)
        self.dron_pos_sub = self.create_subscription(
            Vector3, '/dron/position', self.dron_pos_callback, 10)

        # Publicadores
        self.pos_pub = self.create_publisher(Vector3, '/misil/position', 10)
        self.dir_pub = self.create_publisher(Vector3, '/misil/direction', 10)

        self.rate = self.create_rate(1 / self.dt)  # Usamos rclpy.rate en lugar de rospy.Rate

        self.timer = self.create_timer(self.dt, self.update_state)

    def reset_callback(self, msg):
        if msg.data:
            self.get_logger().info("Reset recibido: reseteando posición y velocidad.")
            self.misil_pos = np.random.uniform(0, self.Tam, size=(3,))

    def dron_pos_callback(self, msg):
        self.dron_pos = np.array([msg.x, msg.y, msg.z])
        

    def update_state(self):
        # Calcular vector de dirección hacia el dron
        target_direction = self.dron_pos - self.misil_pos
        if np.linalg.norm(target_direction) > 0:  # Evitar división por cero
            target_direction /= np.linalg.norm(target_direction)

            # Producto punto para calcular ángulo entre los vectores
            dot = np.clip(np.dot(self.missile_dir, target_direction), -1.0, 1.0)
            angle_between = np.arccos(dot)

            # Ajustar la dirección del misil
            if angle_between < self.missile_turn_rate:
                new_dir = target_direction
            else:
                new_dir = self.missile_dir + (target_direction - self.missile_dir) * (self.missile_turn_rate / angle_between)
                new_dir /= np.linalg.norm(new_dir)

            self.missile_dir = new_dir

            # Publicar dirección del misil
            dir_msg = Vector3(x=float(self.missile_dir[0]), y=float(self.missile_dir[1]), z=float(self.missile_dir[2]))
            self.dir_pub.publish(dir_msg)

            # Actualizar la posición
            self.misil_pos += self.missile_dir * self.missile_speed * self.dt

            # Limitar dentro del entorno
            self.misil_pos = np.clip(self.misil_pos, 0, self.Tam)

            # Publicar posición
            pos_msg = Vector3(x=float(self.misil_pos[0]), y=float(self.misil_pos[1]), z=float(self.misil_pos[2]))
            self.pos_pub.publish(pos_msg)


def main(args=None):
    rclpy.init(args=args)

    misil = MisilModel()
    rclpy.spin(misil)

    rclpy.shutdown()

if __name__ == '__main__':
    main()