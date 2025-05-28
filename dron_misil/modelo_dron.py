#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool
import numpy as np
import time

class DroneModel(Node):
    def __init__(self):
        super().__init__('drone_model_node')
        self.last_reset_time = 0
        self.reset_cooldown = 0.2
        self.dt = 0.1  # intervalo de tiempo
        self.max_speed = 25.0  # velocidad máxima
        self.Tam = 300.0  # tamaño del entorno

        self.initial_position = np.array([self.Tam / 2, self.Tam / 2, self.Tam / 2], dtype=np.float32)

        self.position = np.copy(self.initial_position)
        self.velocity = np.array([0.0, 0.0, 0.0], dtype=np.float32)

        # Suscriptor a la aceleración
        self.acc_sub = self.create_subscription(
            Vector3, '/dron/acceleration', self.acc_callback, 10)

        # Suscriptor al reset
        self.reset_sub = self.create_subscription(
            Bool, '/dron/reset', self.reset_callback, 10)

        # Publicadores de posición y velocidad
        self.pos_pub = self.create_publisher(Vector3, '/dron/position', 10)
        self.vel_pub = self.create_publisher(Vector3, '/dron/velocity', 10)

        self.acceleration = np.array([0.0, 0.0, 0.0], dtype=np.float32)

        self.rate = self.create_rate(1 / self.dt)  # Usamos rclpy.rate en lugar de rospy.Rate

        # Inicializar el timer para la actualización (ahora en el nodo)
        self.timer = self.create_timer(self.dt, self.update_state)

    def acc_callback(self, msg):
        self.acceleration =  np.array([msg.x, msg.y, msg.z], dtype=np.float32)
        

         
    def reset_callback(self, msg):
        if msg.data:
            current_time = time.time()
            if current_time - self.last_reset_time > self.reset_cooldown:
                self.get_logger().info("Reset recibido: reseteando posición y velocidad.")
                self.position = np.copy(self.initial_position)
                self.velocity = np.array([0.0, 0.0, 0.0], dtype=np.float32)
                pos_msg = Vector3(x=float(self.position[0]), y=float(self.position[1]), z=float(self.position[2]))
                self.pos_pub.publish(pos_msg)
                self.last_reset_time = current_time
            else:
                self.get_logger().info("Reset ignorado: esperando cooldown.")
            

    def update_state(self):
        #while rclpy.ok():
            # Integrar la aceleración
            self.velocity += self.acceleration * self.dt
            self.velocity = np.clip(self.velocity, -self.max_speed, self.max_speed)

            # Integrar la velocidad
            self.position += self.velocity * self.dt

            # Limitar la posición dentro del entorno
            self.position = np.clip(self.position, 0.0, self.Tam)

            # Publicar posición y velocidad
            pos_msg = Vector3(x=float(self.position[0]), y=float(self.position[1]), z=float(self.position[2]))
            vel_msg = Vector3(x=float(self.velocity[0]), y=float(self.velocity[1]), z=float(self.velocity[2]))

            self.pos_pub.publish(pos_msg)
            self.vel_pub.publish(vel_msg)

            #self.rate.sleep()  # Frecuencia con rclpy

def main(args=None):
    rclpy.init(args=args)

    drone = DroneModel()
    #drone.run()
    rclpy.spin(drone)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
