from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="scene_management", 
            executable="scenario_manager.py",    
            name="scenario_manager",
            output="screen",
            parameters=[],                        
            remappings=[],                        
        )
    ])
