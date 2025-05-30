#!/usr/bin/env python3
import gymnasium as gym
from gymnasium import spaces
import numpy as np
from vpython import *
import pygame
from pygame.locals import DOUBLEBUF, OPENGL, QUIT, KEYDOWN, K_LEFT, K_RIGHT, K_UP, K_DOWN, K_w, K_s, K_a, K_d, K_q, K_e
import rclpy
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool
from rclpy.node import Node

import time
import os
import sys

current_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(current_dir)

from renderer3D import Advanced3DRenderer  # Importa el renderer 3D


class DroneEvadeSim(gym.Env):
    def __init__(self, render_mode=None):
        super().__init__()
        self.reset_done = False
        self.dt = 0.1
        self.max_speed = 30  #10
        self.max_steps = 1200  #700
        self.Tam = 300.0  # 1000

        self.render_mode = render_mode
        self.scene = None
        self.dron = None
        self.misil = None

        self.dron_pos = np.array([self.Tam / 2, self.Tam / 2, self.Tam / 2])
        self.dron_vel = np.array([0.0, 0.0, 0.0])
        self.misil_pos = np.random.uniform(0, self.Tam, size=(3,))
        self.misil_dir = np.array([0.0, 0.0, 0.0])

        # [dron_x, dron_y, dron_z, misil_x, misil_y, misil_z, vel_dron_x, vel_dron_y, vel_dron_z]
        self.observation_space = spaces.Box(low=-self.Tam, high=self.Tam, shape=(15,), dtype=np.float32)

        # [acc_x, acc_y, acc_z]
        self.action_space = spaces.Box(low=-10.0, high=10.0, shape=(3,), dtype=np.float32)

        # ROS 2 Publishers and Subscribers
        self.node = rclpy.create_node('drone_evade_sim')

        # Publishers
        self.acc_pub = self.node.create_publisher(Vector3, '/dron/acceleration', 10)
        self.reset_pub = self.node.create_publisher(Bool, '/dron/reset', 10)


        # Subscribers
        self.pos_sub_drone = self.node.create_subscription(Vector3, '/dron/position', self.dron_pos_callback, 10)
        self.vel_sub_drone = self.node.create_subscription(Vector3, '/dron/velocity', self.dron_vel_callback, 10)
        self.pos_sub_missile = self.node.create_subscription(Vector3, '/misil/position', self.misil_pos_callback, 10)
        self.dir_sub_missile = self.node.create_subscription(Vector3, '/misil/direction', self.misil_dir_callback, 10)
        self.reset_misil_done = self.node.create_subscription(Vector3, '/misil/reset_misil', self.misil_reset, 10)

        self.reset_misil_done = False  # Estado de reset del misil
        if self.render_mode=="prueba":
            self.renderer = Advanced3DRenderer()

        self.reset(seed=43)


    def misil_reset(self, msg):
        """Callback para el reset del misil"""
        self.reset_misil_done = True

        self.node.get_logger().info("Misil reseteado y en posición inicial.")
    def dron_pos_callback(self, msg):
        """Callback para la posición del dron """
        self.dron_pos = np.array([msg.x, msg.y, msg.z])
        pos_inicial = np.array([150.0, 150.0, 150.0], dtype=np.float32)
        #self.node.get_logger().info(f"Posicion dron:{self.dron_pos}")
        if np.allclose(self.dron_pos, pos_inicial, atol=3.0):
            self.reset_done = True
            #self.node.get_logger().info("Polluelo en posicion inicial, reset_done activado.")

    def misil_pos_callback(self, msg):
        """Callback para la posición del misil (recibida del misil)"""
        self.misil_pos = np.array([msg.x, msg.y, msg.z])
        
    def dron_vel_callback(self, msg):
        """Callback para la velocidad del dron (recibida del misil)"""
        self.dron_vel = np.array([msg.x, msg.y, msg.z])
    
    def misil_dir_callback(self, msg):
        self.misil_dir = np.array([msg.x, msg.y, msg.z])

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)

        self.steps = 0
        obs = self._get_obs()
        self.reset_done = False  # Reiniciar el estado de reset
        self.reset_misil_done = False
        reset_msg = Bool()
        reset_msg.data = True  # Cambia a False si no quieres activar el reset

        # Publicar el mensaje
        self.reset_pub.publish(reset_msg)
        
        
        timeout = 1  # Tiempo máximo de espera para reset_done
        start_time = time.time()
        while not self.reset_done and not self.reset_misil_done:
            if time.time() - start_time > timeout:
                print("Timeout esperando reset_done, saliendo del reset.")
                break
            #time.sleep(0.1)
            rclpy.spin_once(self.node, timeout_sec=0.01)
            
        self.reset_done = False  # Reiniciar el estado de reset

        return obs, {}

    def _get_obs(self):
        distancia = np.array([min(self.dron_pos[0], self.Tam - self.dron_pos[0]),
                              min(self.dron_pos[1], self.Tam - self.dron_pos[1]),
                              min(self.dron_pos[2], self.Tam - self.dron_pos[2])])
        return np.concatenate([self.dron_pos, self.misil_pos, self.dron_vel, distancia, self.misil_dir])

    def step(self, action):
        self.steps += 1

        action = np.clip(action, -15, 15)

        rclpy.spin_once(self.node, timeout_sec=0.01)

        acc_msg = Vector3(x=float(action[0]), y=float(action[1]), z=float(action[2]))

        self.acc_pub.publish(acc_msg)

        dist = np.linalg.norm(self.dron_pos - self.misil_pos)

        # Limitar la posición del dron para que no se salga de los límites
        
        self.dron_pos = np.clip(self.dron_pos, 0, self.Tam)

        # Limitar la posición del misil para que no se salga de los límites
        terminated1=False
        terminated2=False
        terminated = False
        self.misil_pos = np.clip(self.misil_pos, 0, self.Tam)

        terminated1 = dist < 3.5  # debería ser 5.5 asumiendo orientación correcta

        if np.any((self.dron_pos <= 0) | (self.dron_pos >= self.Tam)):
            terminated2 = True

        truncated = self.steps >= self.max_steps

        def survival_reward(t, T):
            '''
            Calcula la recompensa instantánea por sobrevivir en el paso t,
            con T pasos totales. La suma total será 1 si sobrevive hasta el final.
            '''
            total_sum = T * (T + 1) / 2
            return (t + 1) / total_sum

        reward = survival_reward(self.steps, self.max_steps) 
        if terminated1 :
            reward=-0.5 # reward 1.0
            terminated = True

        if terminated2:
            terminated = True
            reward = -1.0

        if truncated:
            reward = 0.5  # 0.5

        reward=(1-2*(self.steps/self.max_steps))/self.max_steps
        
        if np.any((self.misil_pos <= 0) | (self.misil_pos >= self.Tam)):  # si se ha salido de los límites
            truncated = True
            reward = 1.1 - self.steps / self.max_steps

        if self.render_mode == "human":
            self.render()

        
        if self.render_mode=="prueba":
            events = pygame.event.get() 
        




            # Iterar sobre los eventos para el QUIT
            for event in events:
                if event.type == QUIT:
                 running = False
            self.renderer.render(self.dron_pos, self.misil_pos,self.misil_dir,events)
        elif self.render_mode== "Pybullet":
            self.renderer.render(self.dron_pos, self.misil_pos,self.misil_dir)
        return self._get_obs(), reward, terminated, truncated, {}

    def render(self):
        if self.scene is None:
            # Crear la escena
            self.scene = canvas(title="Simulación: Habitación con dron y misil", width=800, height=600)

            # Tamaño de la habitación (un cubo de 200x200x200)
            habitacion = box(pos=vector(self.Tam / 2, self.Tam / 2, self.Tam / 2), size=vector(self.Tam, self.Tam, self.Tam), opacity=0.1, color=color.white)

            # Dron (una esfera azul)
            self.dron = sphere(pos=vector(10, 12, 0), radius=2.5, color=color.cyan, make_trail=False)

            # Misil (un prisma rojo, hecho con un cilindro largo y una punta)
            self.misil = box(pos=vector(0, 1, 0), size=vector(20, 2, 2), color=color.red, make_trail=False)
            self.scene.camera.pos = vector(self.Tam + 50, self.Tam + 50, self.Tam + 150)  # La cámara está detrás y por encima
            self.scene.camera.axis = vector(-100, -100, -100)

        # Convertir coordenadas al centro de la pantalla
        def convert(pos):
            return int(self.size / 2 + pos[0] * self.scale), int(self.size / 2 - pos[1] * self.scale)

        pos = self.dron_pos
        self.dron.pos.x = int(pos[0])
        self.dron.pos.y = int(pos[1])
        self.dron.pos.z = int(pos[2])

        pos = self.misil_pos
        self.misil.pos.x = int(pos[0])
        self.misil.pos.y = int(pos[1])
        self.misil.pos.z = int(pos[2])
        self.misil.axis = vector(self.misil_dir[0], self.misil_dir[1], self.misil_dir[2]) * 6

        rate(100)

    def close(self):
        if self.scene:
            print("Se acabó")
            # sys.exit("Fin de la simulación")



