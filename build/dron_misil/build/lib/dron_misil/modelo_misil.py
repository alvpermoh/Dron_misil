#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool
import numpy as np

class MisilModel:
    def __init__(self):
        rospy.init_node('misil_model_node', anonymous=True)

        self.dt = 0.1  # intervalo de tiempo
        self.max_speed = 10.0  # velocidad máxima
        self.Tam = 1000.0  # tamaño del entorno

        self.misil_pos = np.random.uniform(0, self.Tam, size=(3,))

        self.missile_speed = 3  # Velocidad del misil (ajustable)
        self.missile_turn_rate = 0.1  # Tasa de giro del misil (ajustable)

        self.missile_dir = np.array([1.0, 0.0, 0.0])  # Dirección inicial (por ejemplo hacia X)

        self.dron_pos = np.random.uniform(0, self.Tam, size=(3,))  # Inicializar alguna posición para evitar errores

        # Suscriptores
        self.reset_sub = rospy.Subscriber('/dron/reset', Bool, self.reset_callback)
        self.dron_pos_sub = rospy.Subscriber('/dron/position', Vector3, self.dron_pos_callback)

        # Publicadores
        self.pos_pub = rospy.Publisher('/misil/position', Vector3, queue_size=10)

        self.rate = rospy.Rate(1/self.dt)

    def reset_callback(self, msg):
        if msg.data:
            rospy.loginfo("Reset recibido: reseteando posición y velocidad.")
            self.misil_pos = np.random.uniform(0, self.Tam, size=(3,))

    def dron_pos_callback(self, msg):
        self.dron_pos = np.array([msg.x, msg.y, msg.z])

    def run(self):
        while not rospy.is_shutdown():
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

                # Actualizar la posición
                self.misil_pos += self.missile_dir * self.missile_speed * self.dt

                # Limitar dentro del entorno
                self.misil_pos = np.clip(self.misil_pos, 0, self.Tam)

                # Publicar posición
                pos_msg = Vector3(x=self.misil_pos[0], y=self.misil_pos[1], z=self.misil_pos[2])
                self.pos_pub.publish(pos_msg)

            self.rate.sleep()

if __name__ == '__main__':
    try:
        misil = MisilModel()
        misil.run()
    except rospy.ROSInterruptException:
        pass
