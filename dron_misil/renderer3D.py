import pygame
from pygame.locals import DOUBLEBUF, OPENGL, QUIT, KEYDOWN, K_LEFT, K_RIGHT, K_UP, K_DOWN, K_w, K_s, K_a, K_d, K_q, K_e

from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *

import os
import numpy as np

# Suprimir advertencias de ALSA
os.environ['SDL_AUDIODRIVER'] = 'dummy'

class Advanced3DRenderer:
    def __init__(self):
        pygame.init()
        glutInit()

        self.screen_width = 1920
        self.screen_height = 1080
        self.screen = pygame.display.set_mode((self.screen_width, self.screen_height), DOUBLEBUF | OPENGL)
        pygame.display.set_caption("Dron y Misil - Parámetros Externos")

        glEnable(GL_DEPTH_TEST)

        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45, (self.screen_width / self.screen_height), 0.1, 1000.0)

        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        glViewport(0, 0, self.screen_width, self.screen_height)

        # --- Posición y Rotación Inicial de la Cámara ---
        self.camera_x = -400.0 # Ajustado para ver el suelo -300
        self.camera_y = -430.0 # Ajustado para ver el suelo -300
        self.camera_z = -630.0 # Ajustado para ver el suelo -900

        self.rotation_x = 25 # Inclinado para ver el suelo 10
        self.rotation_y = -30 # Rotación inicial de la cámara -25

    def draw_dron(self, pos, size=15):
        x, y, z = pos
        glPushMatrix()
        glTranslatef(x, y, z)
        glScalef(size, size, size)

        # Cuerpo central (rectángulo/cubo achatado)
        glColor3f(0.2, 0.2, 0.2)  # Gris oscuro
        glPushMatrix()
        glScalef(0.5, 0.12, 0.5)
        glutSolidCube(1.0)
        glPopMatrix()

        # Brazos en X
        glColor3f(0.4, 0.4, 0.4)  # Gris medio
        for angle in [0, 90, 180, 270]:
            glPushMatrix()
            glRotatef(angle, 0, 1, 0)
            glTranslatef(0.4, 0, 0)
            glScalef(0.8, 0.07, 0.07)
            glutSolidCube(1.0)
            glPopMatrix()

        # Rotores (discos)
        glColor3f(0.05, 0.05, 0.05)  # Casi negro
        for angle in [0, 90, 180, 270]:
            glPushMatrix()
            glRotatef(angle, 0, 1, 0)
            glTranslatef(0.8, 0.08, 0)
            glRotatef(90, 1, 0, 0)
            glutSolidTorus(0.04, 0.13, 16, 24)
            glPopMatrix()


        glPopMatrix()

    def draw_misil(self, pos, orientation, size=10):
        x, y, z = pos
        # orientation ahora es un vector dirección (dx, dy, dz)
        if len(orientation) == 3:
            dir_vec = np.array(orientation, dtype=float)
            norm = np.linalg.norm(dir_vec)
            if norm == 0:
                dir_vec = np.array([0, 1, 0])
            else:
                dir_vec = dir_vec / norm
            # Calcula ángulo y eje para alinear el misil con la dirección
            # Por defecto el misil apunta a +Y, así que rotamos de [0,1,0] a dir_vec
            up = np.array([0, 1, 0])
            axis = np.cross(up, dir_vec)
            angle = np.arccos(np.clip(np.dot(up, dir_vec), -1.0, 1.0)) * 180.0 / np.pi
        else:
            # fallback: usar pitch, yaw, roll
            pitch, yaw, roll = orientation
            axis = [0, 0, 1]
            angle = 0

        glPushMatrix()
        glTranslatef(x, y, z)
        if 'angle' in locals() and np.linalg.norm(axis) > 1e-6 and angle != 0:
            glRotatef(angle, axis[0], axis[1], axis[2])
        elif len(orientation) == 3:
            pass  # ya está alineado
        else:
            glRotatef(roll, 0, 0, 1)
            glRotatef(yaw, 0, 1, 0)
            glRotatef(pitch, 1, 0, 0)
        glScalef(size, size, size)

        # Cuerpo principal (cilindro)
        glColor3f(0.5, 0.0, 0.0)  # Rojo oscuro
        glPushMatrix()
        glRotatef(90, 1, 0, 0)
        glutSolidCylinder(0.13, 1.2, 20, 20)
        glPopMatrix()

        # Punta (cono) perfectamente pegada
        glColor3f(1.0, 0.1, 0.1)  # Rojo intenso
        glPushMatrix()
        glTranslatef(0, 0, 0)  # extremo del cilindro, con epsilon para evitar z-fighting
        glRotatef(-90, 1, 0, 0)
        glutSolidCone(0.13, 0.32, 20, 20)
        glPopMatrix()

        glPopMatrix()

    def draw_ground(self, size=3000):
        """Dibuja un plano de suelo grande y verde en el plano XZ."""
        glPushMatrix()
        # El plano XZ está en el "suelo" en y=0
        x_min, z_min = -size//2, -size//2
        x_max, z_max = size//2, size//2

        # Cambia el color a verde para el suelo
        glColor3f(0.1, 0.7, 0.1)

        glBegin(GL_QUADS)
        glVertex3f(x_min, 0.0, z_min)
        glVertex3f(x_max, 0.0, z_min)
        glVertex3f(x_max, 0.0, z_max)
        glVertex3f(x_min, 0.0, z_max)
        glEnd()

        # Opcional: Dibuja una rejilla en el suelo para visualización
        glColor3f(0.2, 0.4, 0.2) # Verde más oscuro para la rejilla
        glBegin(GL_LINES)
        step = 50
        for i in range(x_min, x_max + 1, step):
            glVertex3f(i, 0.01, z_min)
            glVertex3f(i, 0.01, z_max)
        for j in range(z_min, z_max + 1, step):
            glVertex3f(x_min, 0.01, j)
            glVertex3f(x_max, 0.01, j)
        glEnd()

        glPopMatrix()

    def render(self, dron_pos, misil_pos, misil_orientation, events):
        # Manejo de eventos de teclado para la cámara
        for event in events:
            if event.type == QUIT:
                pygame.quit()
                quit()
            if event.type == KEYDOWN:
                # Mover la cámara (mover el mundo en dirección opuesta)
                if event.key == K_LEFT: self.camera_x += 10
                if event.key == K_RIGHT: self.camera_x -= 10
                if event.key == K_UP: self.camera_y -= 10
                if event.key == K_DOWN: self.camera_y += 10
                if event.key == K_w: self.camera_z += 10
                if event.key == K_s: self.camera_z -= 10

                # Rotar la vista
                if event.key == K_a: self.rotation_y -= 5
                if event.key == K_d: self.rotation_y += 5
                if event.key == K_q: self.rotation_x -= 5
                if event.key == K_e: self.rotation_x += 5


        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glClearColor(0.5, 0.8, 1.0, 1.0)  # Celeste

        glLoadIdentity()

        glRotatef(self.rotation_x, 1, 0, 0)
        glRotatef(self.rotation_y, 0, 1, 0)

        glTranslatef(self.camera_x, self.camera_y, self.camera_z)

        # --- Dibuja el suelo primero ---
        self.draw_ground()  # Llama a la nueva función para dibujar el suelo grande

        self.draw_dron(dron_pos)
        self.draw_misil(misil_pos, misil_orientation)

        pygame.display.flip()
        pygame.time.wait(10)

        error = glGetError()
        if error != GL_NO_ERROR:
            print(f"OpenGL Error: {gluErrorString(error).decode()}")

# --- Punto de entrada principal ---
if __name__ == "__main__":
    renderer = Advanced3DRenderer()

    current_dron_pos = [150.0, 150.0, 150.0] # Ajustado para que el dron esté por encima del suelo
    current_misil_pos = [50.0, 20.0, 10.0]
    current_misil_orientation = [30.0, 45.0, 0.0]

    running = True
    while running:
        # --- CLAVE: Obtener los eventos una sola vez en el bucle principal ---
        events = pygame.event.get()

        # Iterar sobre los eventos para el QUIT
        for event in events:
            if event.type == QUIT:
                running = False

        # Pasa la lista de eventos al renderizador
        renderer.render(current_dron_pos, current_misil_pos, current_misil_orientation, events)

        # Opcional: Para ver movimiento, puedes actualizar estas variables aquí
        #current_dron_pos[2] += 1
        #if current_dron_pos[0] > 100: current_dron_pos[0] = -100
        current_misil_orientation[1] += 0.5
        print("Posición de la camara x,y,z:",renderer.camera_x,renderer.camera_y,renderer.camera_z, "Rotacion de la camara x,y",renderer.rotation_x, renderer.rotation_y)

    pygame.quit()
    quit()
