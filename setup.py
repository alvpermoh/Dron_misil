from setuptools import setup
import os
from glob import glob
package_name = 'dron_misil'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packagetype', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/dron_misil_launch.py']),
                # --- AÑADE ESTAS LÍNEAS ---
        # Ruta de destino: install/<pkg_name>/share/<pkg_name>/mejores_modelos
        # Ruta de origen: src/<pkg_name>/<pkg_name>/mejores_modelos
        (os.path.join('share', package_name, 'mejores_modelos'), glob(os.path.join(package_name, 'mejores_modelos', '*.*'))),
        # Si tienes subcarpetas dentro de mejores_modelos, necesitarías una estrategia más avanzada
        # como usar 'include_package_data=True' con MANIFEST.in, o copiar la carpeta completa.
        # Para una sola carpeta de modelos, glob('*.*') suele bastar si solo hay archivos.
        # Si 'best_model.zip' está directamente en 'mejores_modelos', esto funciona.
        # --- FIN AÑADIDO ---
    ],
    install_requires=[
        'setuptools',      # Asegúrate de que setuptools esté allí
        'rclpy',           # Si estás usando rclpy
        'launch',          # Si usas launch
        'numpy',           # Si usas numpy o cualquier otra librería externa
        'requests',        # Si usas requests o similares
        # Agrega más dependencias necesarias
    ],
    zip_safe=True,
    maintainer='Tu Nombre',
    maintainer_email='tu_email@dominio.com',
    description='Simulación de drones, misiles y entorno',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'modelo_dron = dron_misil.modelo_dron:main',
            'modelo_misil = dron_misil.modelo_misil:main',
            'simulacion_dron_misil = dron_misil.simulacion_dron_misil:main',
        ],
    },
)
