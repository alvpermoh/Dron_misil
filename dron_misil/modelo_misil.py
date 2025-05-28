#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool
import numpy as np
import time
class MisilModel(Node):
    def __init__(self):
        super().__init__('misil_model_node')
        self.last_reset_time = 0
        self.reset_cooldown = 0.2
        self.dt = 0.1  # intervalo de tiempo
        self.max_speed = 20.0  # velocidad máxima
        self.Tam = 300.0  # tamaño del entorno

        self.misil_pos = np.random.uniform(0, self.Tam, size=(3,))

        self.missile_speed = 25  # Velocidad del misil (ajustable)
        self.missile_turn_rate = 0.2  # Tasa de giro del misil (ajustable)
        self.missile_dir = np.array([1.0, 0.0, 0.0])  # Dirección inicial
        self.dron_pos = np.random.uniform(0, self.Tam, size=(3,))
        self.dron_vel = np.zeros(3, dtype=np.float32)  # Velocidad estimada del dron
        self.last_dron_pos = self.dron_pos.copy()
        self.last_dron_update = time.time()

        # --- Dinámica física avanzada ---
        self.mass = 1.0  # masa del misil en kg
        self.drag_coef = 0.10  # coeficiente de rozamiento
        self.max_acc = 20.0  # aceleración máxima (m/s^2)
        self.gravity = np.array([0.0, 0.0, -9.81], dtype=np.float32)  # gravedad en Z
        self.acc_noise_std = 0.3  # desviación estándar del ruido en aceleración
        self.motor_tau = 0.1  # constante de tiempo del motor
        self.missile_velocity = self.missile_dir * self.missile_speed
        self.target_acceleration = np.zeros(3, dtype=np.float32)

        # Suscriptores
        self.reset_sub = self.create_subscription(
            Bool, '/dron/reset', self.reset_callback, 10)
        self.dron_pos_sub = self.create_subscription(
            Vector3, '/dron/position', self.dron_pos_callback, 10)

        # Publicadores
        self.pos_pub = self.create_publisher(Vector3, '/misil/position', 10)
        self.dir_pub = self.create_publisher(Vector3, '/misil/direction', 10)
        self.reset_pub = self.create_publisher(Bool, '/misil/reset_misil', 10)

        self.rate = self.create_rate(1 / self.dt)
        self.timer = self.create_timer(self.dt, self.update_state)

    def reset_callback(self, msg):
        if msg.data:
            current_time = time.time()
            if current_time - self.last_reset_time > self.reset_cooldown:
                
                self.misil_pos = np.random.uniform(0, self.Tam, size=(3,))
                self.missile_velocity = self.missile_dir * self.missile_speed
                self.target_acceleration = np.zeros(3, dtype=np.float32)
                pos_msg = Vector3(x=float(self.misil_pos[0]), y=float(self.misil_pos[1]), z=float(self.misil_pos[2]))
                self.pos_pub.publish(pos_msg)
                self.reset_pub.publish(Bool(data=True))  # Publicar reset
                self.get_logger().info(f"Misil reseteado a posición: {self.misil_pos}")
                self.last_reset_time = current_time
                
            else:
                self.get_logger().info("Reset ignorado: esperando cooldown.")
            

    def dron_pos_callback(self, msg):
        new_pos = np.array([msg.x, msg.y, msg.z])
        now = time.time()
        dt = now - self.last_dron_update if self.last_dron_update else self.dt
        if dt > 0:
            self.dron_vel = (new_pos - self.last_dron_pos) / dt
        self.last_dron_pos = new_pos
        self.last_dron_update = now
        self.dron_pos = new_pos

    def update_state(self):
        # --- Pure Pursuit mejorado: apunta a la posición futura del dron ---
        pursuit_time = 0.5  # segundos en el futuro a predecir
        predicted_dron_pos = self.dron_pos + self.dron_vel * pursuit_time
        target_direction = predicted_dron_pos - self.misil_pos
        if np.linalg.norm(target_direction) > 0:
            target_direction /= np.linalg.norm(target_direction)
            dot = np.clip(np.dot(self.missile_dir, target_direction), -1.0, 1.0)
            angle_between = np.arccos(dot)
            if angle_between < self.missile_turn_rate:
                new_dir = target_direction
            else:
                new_dir = self.missile_dir + (target_direction - self.missile_dir) * (self.missile_turn_rate / angle_between)
                new_dir /= np.linalg.norm(new_dir)
            self.missile_dir = new_dir

            # --- Dinámica física avanzada ---
            desired_acc = (self.missile_dir * self.missile_speed - self.missile_velocity) / self.dt
            desired_acc = np.clip(desired_acc, -self.max_acc, self.max_acc)
            alpha = self.dt / (self.motor_tau + self.dt)
            self.target_acceleration = (1 - alpha) * self.target_acceleration + alpha * desired_acc
            noisy_acc = self.target_acceleration + np.random.normal(0, self.acc_noise_std, size=3)
            total_acc = noisy_acc + self.gravity
            drag = -self.drag_coef * self.missile_velocity
            self.missile_velocity += (total_acc + drag) * self.dt / self.mass
            speed = np.linalg.norm(self.missile_velocity)
            if speed > self.missile_speed:
                self.missile_velocity = self.missile_velocity / speed * self.missile_speed
            self.misil_pos += self.missile_velocity * self.dt
            # Colisión con el suelo (Z=0)
            if self.misil_pos[2] < 0.0:
                self.misil_pos[2] = 0.0
                self.missile_velocity[2] = 0.0
            # Limitar dentro del entorno
            self.misil_pos = np.clip(self.misil_pos, 0, self.Tam)

            # Publicar dirección y posición
            dir_msg = Vector3(x=float(self.missile_dir[0]), y=float(self.missile_dir[1]), z=float(self.missile_dir[2]))
            self.dir_pub.publish(dir_msg)
            pos_msg = Vector3(x=float(self.misil_pos[0]), y=float(self.misil_pos[1]), z=float(self.misil_pos[2]))
            self.pos_pub.publish(pos_msg)


def main(args=None):
    rclpy.init(args=args)

    misil = MisilModel()
    rclpy.spin(misil)

    rclpy.shutdown()

if __name__ == '__main__':
    main()