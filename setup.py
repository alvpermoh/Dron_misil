from setuptools import setup

package_name = 'dron_misil'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packagetype', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/dron_misil_launch.py']),
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
