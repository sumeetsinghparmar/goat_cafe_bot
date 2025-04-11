import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = "[{name}]: {message}"


def generate_launch_description():

    order_delivery_srv = Node(
        package="ebot_nav2",
        name="delivery_srv",
        executable="order_delivery_service.py",
        output='screen'
    )

    confirmation_srv = Node(
        package="ebot_nav2",
        name="confirmation_srv",
        executable="confirmation_service.py",
        output='screen'
    )
    
    o_or_c_service = Node(
        package="ebot_nav2",
        name="order_or_cancel_srv",
        executable="order_or_cancel_service.py",
        output='screen'
    )

    state_updater_srv = Node(
        package="ebot_nav2",
        name="state_updater_srv",
        executable="state_updater_service.py",
        output='screen'
    )

    return LaunchDescription([
        TimerAction(period=0.0, actions=[confirmation_srv]),
        TimerAction(period=1.0, actions=[o_or_c_service]),
        TimerAction(period=3.0, actions=[state_updater_srv]),
        TimerAction(period=5.0, actions=[order_delivery_srv])
    ])
