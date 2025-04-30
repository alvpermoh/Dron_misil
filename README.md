# 🧪 Entorno de Simulación con Chrome y Python

Este proyecto utiliza Python para crear entornos de simulación visual con `gymnasium`, `vpython`, y `numpy`, renderizados a través de Google Chrome desde un contenedor.

---

## 📦 Requisitos: Instalación de librerías Python

Ejecuta este comando para instalar las dependencias necesarias:

```bash
pip install gymnasium vpython numpy
´´´

# Descargar el paquete .deb de Chrome
```bash
wget https://dl.google.com/linux/direct/google-chrome-stable_current_amd64.deb
´´´
# Instalar el paquete .deb
```bash
sudo dpkg -i google-chrome-stable_current_amd64.deb
´´´
# Solucionar dependencias si fuera necesario
```bash
sudo apt --fix-broken install -y
´´´
#compilar paquete
```bash
colcon build
source install/setup.zsh
´´´
#lanzar
```bash
ros2 launch dron_misil dron_misil_launch.py
´´´

