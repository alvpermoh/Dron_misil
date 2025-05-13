#!/usr/bin/env python3
import gymnasium as gym
from gymnasium import spaces
import numpy as np
from vpython import *

import rospy
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool




class DroneEvadeSim(gym.Env):
    def __init__(self, render_mode=None):
        super().__init__()
        self.dt = 0.1
        self.max_speed = 10  #10
        self.max_steps = 2000  #700
        self.Tam=1000.0         #300

        self.render_mode = render_mode
        self.scene = None
        self.dron = None
        self.misil=None
        
        self.dron_pos = np.array([self.Tam/2, self.Tam/2,self.Tam/2])
        self.dron_vel = np.array([0.0, 0.0, 0.0])
        self.misil_pos = np.random.uniform(0,self.Tam, size=(3,))
        self.misil_dir= np.array([0.0, 0.0, 0.0])
        
        

        # [dron_x, dron_y,dron_z, misil_x, misil_y,misil_z, vel_dron_x, vel_dron_y.vel_dron_z]
        self.observation_space = spaces.Box(low=-self.Tam, high=self.Tam, shape=(15,), dtype=np.float32)

        # [acc_x, acc_y,acc_z]
        self.action_space = spaces.Box(low=-10.0, high=10.0, shape=(3,), dtype=np.float32)


        
        # Publicadores
        self.acc_pub = rospy.Publisher('/dron/acceleration', Vector3,queue_size=10)
        self.reset_pub = rospy.Publisher('/dron/reset', Bool, queue_size=10)
        

        # Suscriptores
        self.pos_sub_drone = rospy.Subscriber('/dron/position', Vector3, self.dron_pos_callback)
        self.vel_sub_drone = rospy.Subscriber('/dron/velocity', Vector3, self.dron_vel_callback)
        self.pos_sub_missile = rospy.Subscriber('/misil/position', Vector3, self.misil_pos_callback)


        self.reset(seed=43)
    def dron_pos_callback(self, msg):
        """Callback para la posición del dron (recibida del misil)"""
        self.dron_pos = np.array([msg.x, msg.y, msg.z])
    def misil_pos_callback(self, msg):
        """Callback para la posición del dron (recibida del misil)"""
        self.misil_pos = np.array([msg.x, msg.y, msg.z])
    def dron_vel_callback(self, msg):
        """Callback para la posición del dron (recibida del misil)"""
        self.dron_vel = np.array([msg.x, msg.y, msg.z])

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        
        self.steps = 0
        obs = self._get_obs()

        reset_msg = Bool()
        reset_msg.data = True  # Cambia a False si no quieres activar el reset

        # Publicar el mensaje
        self.reset_pub.publish(reset_msg)
        
        
    
        
        return obs, {}

    def _get_obs(self):
        distancia=np.array([min(self.dron_pos[0],self.Tam-self.dron_pos[0]),min(self.dron_pos[1],self.Tam-self.dron_pos[1]),min(self.dron_pos[2],self.Tam-self.dron_pos[2])])
        return np.concatenate([self.dron_pos, self.misil_pos, self.dron_vel,distancia,self.misil_dir])

    def step(self, action):
        self.steps += 1

        action = np.clip(action, -10, 10)

        acc_msg = Vector3(x=self.action[0], y=self.action[1], z=self.action[2])




        
        self.acc_pub.publish(acc_msg)
        

        dist = np.linalg.norm(self.dron_pos - self.misil_pos)


        # 3. Limitar la posición del dron para que no se salga de los límites
        dron_ant=self.dron_pos
        self.dron_pos = np.clip(self.dron_pos, 
                         0, 
                         self.Tam)
        
        #3Limitar la posición del misil para que no se salga de los límites
        misil_ant=self.misil_pos
        self.misil_pos = np.clip(self.misil_pos, 
                         0, 
                         self.Tam)


        terminated = dist < 0.5  #debería ser 5.5 asumiendo orientación correcta

        if np.any(dron_ant != self.dron_pos):  #si se ha salido de los límites
            terminated=True   
          
        
        truncated = self.steps >= self.max_steps

        def survival_reward(t, T):
            '''
            Calcula la recompensa instantánea por sobrevivir en el paso t,
             con T pasos totales. La suma total será 1 si sobrevive hasta el final.
                '''
            total_sum = T * (T + 1) / 2
            return (t + 1) / total_sum
        
        reward =  survival_reward(self.steps,self.max_steps) if not terminated else -0.5  #reward 1.0  



        if truncated:
            reward=0.5 #0.5

        if np.any(misil_ant != self.misil_pos):  #si se ha salido de los límites
            truncated=True 
            reward=1.1 -self.steps/self.max_steps

        if self.render_mode == "human":
            self.render()

        self.rate.sleep()


        return self._get_obs(), reward, terminated, truncated, {}

    def render(self):
        if self.scene is None:
            # Crear la escena
            self.scene = canvas(title="Simulación: Habitación con dron y misil", width=800, height=600)

            # Tamaño de la habitación (un cubo de 200x200x200)
            habitacion = box(pos=vector(self.Tam/2,self.Tam/2,self.Tam/2), size=vector(self.Tam,self.Tam,self.Tam), opacity=0.1, color=color.white)

            # Dron (una esfera azul)
            self.dron = sphere(pos=vector(10, 12, 0), radius=2.5, color=color.cyan, make_trail=False)

            # Misil (un prisma rojo, hecho con un cilindro largo y una punta)
            #self.misil = box(pos=vector(0, 1, 0), size=vector(2, 6, 2), color=color.red, make_trail=False)
            self.misil = box(pos=vector(0, 1, 0), size=vector(20, 2, 2), color=color.red, make_trail=False)
            #self.scene.camera.follow(self.dron)
            self.scene.camera.pos = vector(self.Tam+50,self.Tam+50, self.Tam+150)  # La cámara está detrás y por encima
            #self.scene.camera.lookat = vector(100,100,100)
            self.scene.camera.axis = vector(-100,-100,-100)



            
        

        # Convertir coordenadas al centro de la pantalla
        def convert(pos):
            return int(self.size / 2 + pos[0] * self.scale), int(self.size / 2 - pos[1] * self.scale)
        
        pos=self.dron_pos
        #pos=convert(self.dron_pos)
        #print(self.dron_pos,self.dron.pos)
        
        self.dron.pos.x=int(pos[0])  #/10
        self.dron.pos.y=int(pos[1]) #/10
        self.dron.pos.z=int(pos[2])

        pos=self.misil_pos

        #pos=convert(self.misil_pos)

        self.misil.pos.x=int(pos[0])
        self.misil.pos.y=int(pos[1])
        self.misil.pos.z=int(pos[2])
        #direction=vector(np.cos(self.current_angle-math.pi/2),np.sin(self.current_angle-math.pi/2),0)
        #self.misil.axis=direction.norm() 
        self.misil.axis=vector(self.misil_dir[0],self.misil_dir[1],self.misil_dir[2])*6
        #print(self.scene.camera.pos)
        #print(self.dron_vel)
        
        rate(100)

        


    def close(self):
        if self.scene:
            print("Se acabó")
            #sys.exit("Fin de la simulación")



'''
env = DroneEvadeEnv3D(render_mode="human")
obs, _ = env.reset()

for _ in range(3000):
    action = env.action_space.sample()  # movimiento aleatorio
    obs, reward, terminated, truncated, _ = env.step(action)
    if terminated or truncated:
        break

env.close()
'''


if __name__ == '__main__':
    try:
        env = DroneEvadeSim(render_mode="human")
        obs, _ = env.reset()

        for _ in range(3000):
            action = env.action_space.sample()  # movimiento aleatorio
            obs, reward, terminated, truncated, _ = env.step(action)
            if terminated or truncated:
                break

        env.close()
    except rospy.ROSInterruptException:
        pass