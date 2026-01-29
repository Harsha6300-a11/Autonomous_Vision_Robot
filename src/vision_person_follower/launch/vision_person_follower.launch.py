from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    perception_node = Node(
        package='vision_person_follower',
        executable='perception_node',
        name='perception_node',
        output='screen'
    )

    controller_node = Node(
        package='vision_person_follower',
        executable='controller_node',
        name='controller_node',
        output='screen'
    )

    return LaunchDescription([
        perception_node,
        controller_node
    ])
