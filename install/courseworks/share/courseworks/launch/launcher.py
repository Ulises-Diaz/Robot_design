import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Ejecutar Nodo Publisher en un terminal separado
        ExecuteProcess(
            cmd=['gnome-terminal', '--', 'ros2', 'run', 'courseworks', 'publisher'],
            output='screen'
        ),

        # Ejecutar Nodo Subscriber en otro terminal separado
        ExecuteProcess(
            cmd=['gnome-terminal', '--', 'ros2', 'run', 'courseworks', 'subscriber'],
            output='screen'
        ),

        # Ejecutar rqt_graph en un terminal
        ExecuteProcess(
            cmd=['gnome-terminal', '--', 'rqt_graph'],
            output='screen'
        ),

        # Ejecutar PlotJuggler en un terminal
        ExecuteProcess(
            cmd=['gnome-terminal', '--', 'plotjuggler'],
            output='screen'
        ),
    ])
