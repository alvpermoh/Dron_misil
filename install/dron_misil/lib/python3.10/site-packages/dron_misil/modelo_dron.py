#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool
import numpy as np

class DroneModel:
    def __init__(self):
        rospy.init_node('drone_model_node', anonymous=True)

        self.dt = 0.1  # intervalo de tiempo
        self.max_speed = 10.0  # velocidad máxima
        self.Tam = 1000.0  # tamaño del entorno

        self.initial_position = np.array([self.Tam/2, self.Tam/2, self.Tam/2], dtype=np.float32)

        self.position = np.copy(self.initial_position)
        self.velocity = np.array([0.0, 0.0, 0.0], dtype=np.float32)

        # Suscriptor a la aceleración
        self.acc_sub = rospy.Subscriber('/dron/acceleration', Vector3, self.acc_callback)

        # Suscriptor al reset
        self.reset_sub = rospy.Subscriber('/dron/reset', Bool, self.reset_callback)

        # Publicadores de posición y velocidad
        self.pos_pub = rospy.Publisher('/dron/position', Vector3, queue_size=10)
        self.vel_pub = rospy.Publisher('/dron/velocity', Vector3, queue_size=10)

        self.acceleration = np.array([0.0, 0.0, 0.0], dtype=np.float32)

        self.rate = rospy.Rate(1/self.dt)

    def acc_callback(self, msg):
        self.acceleration = np.array([msg.x, msg.y, msg.z], dtype=np.float32)

    def reset_callback(self, msg):
        if msg.data:  # Si el mensaje de reset es True
            rospy.loginfo("Reset recibido: reseteando posición y velocidad.")
            self.position = np.copy(self.initial_position)
            self.velocity = np.array([0.0, 0.0, 0.0], dtype=np.float32)

    def run(self):
        while not rospy.is_shutdown():
            # Integrar la aceleración
            self.velocity += self.acceleration * self.dt
            self.velocity = np.clip(self.velocity, -self.max_speed, self.max_speed)

            # Integrar la velocidad
            self.position += self.velocity * self.dt

            # Limitar la posición dentro del entorno
            self.position = np.clip(self.position, 0, self.Tam)

            # Publicar posición y velocidad
            pos_msg = Vector3(x=self.position[0], y=self.position[1], z=self.position[2])
            vel_msg = Vector3(x=self.velocity[0], y=self.velocity[1], z=self.velocity[2])

            self.pos_pub.publish(pos_msg)
            self.vel_pub.publish(vel_msg)

            self.rate.sleep()

if __name__ == '__main__':
    try:
        drone = DroneModel()
        drone.run()
    except rospy.ROSInterruptException:
        pass
