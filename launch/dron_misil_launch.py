import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Lanzar el script del modelo de dron
        Node(
            package='dron_misil', 
            executable='modelo_dron', 
            name='modelo_dron',
            output='screen'
        ),
        # Lanzar el script del modelo de misil
        Node(
            package='dron_misil', 
            executable='modelo_misil', 
            name='modelo_misil',
            output='screen'
        ),
        # Lanzar el entorno
        Node(
            package='dron_misil', 
            executable='simulacion_dron_misil', 
            name='simulacion_dron_misil',
            output='screen'
        ),
    ])