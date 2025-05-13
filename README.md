# 🧪 Entorno de Simulación con Chrome y Python

Este proyecto utiliza Python para crear entornos de simulación visual con `gymnasium`, `vpython` y `numpy`, renderizados a través de Google Chrome, incluso desde un contenedor.

---

## 📦 Requisitos: Instalación de librerías Python

Ejecuta el siguiente comando para instalar las dependencias necesarias:

```bash
pip install gymnasium vpython numpy stable-lines3
```

---

## 🌐 Instalación de Google Chrome (desde .deb)

### Descargar el paquete `.deb` de Chrome

```bash
wget https://dl.google.com/linux/direct/google-chrome-stable_current_amd64.deb
```

### Instalar el paquete `.deb`

```bash
sudo dpkg -i google-chrome-stable_current_amd64.deb
```

### Solucionar dependencias (si fuera necesario)

```bash
sudo apt --fix-broken install -y
```

---

## 🔧 Compilar el paquete ROS 2

Asegúrate de estar en el directorio raíz del workspace de ROS 2:

```bash
colcon build
source install/setup.zsh
```

---

## 🚀 Lanzar el paquete `dron_misil`

```bash
ros2 launch dron_misil dron_misil_launch.py
```

