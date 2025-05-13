import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Lanzar el script del modelo de dron
        Node(
            package='drone_misil', 
            executable='modelo_dron.py', 
            name='modelo_dron',
            output='screen'
        ),
        # Lanzar el script del modelo de misil
        Node(
            package='drone_misil', 
            executable='modelo_misil.py', 
            name='modelo_misil',
            output='screen'
        ),
        # Lanzar el entorno
        Node(
            package='drone_misil', 
            executable='entorno.py', 
            name='entorno',
            output='screen'
        ),
    ])