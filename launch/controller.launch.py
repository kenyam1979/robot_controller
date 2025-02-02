####################################################
#####  Launch file for controller and sensors  #####
####################################################

import os

from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription

from launch import LaunchDescription


import launch
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
import launch_ros



def generate_launch_description():

    controller_pkg_share = launch_ros.substitutions.FindPackageShare(package='robot_controller').find('robot_controller')
    lidar_pkg_share = launch_ros.substitutions.FindPackageShare(package='sllidar_ros2').find('sllidar_ros2')
    imu_pkg_share = launch_ros.substitutions.FindPackageShare(package='wit_ros2_imu').find('wit_ros2_imu')
    description_pkg_share = launch_ros.substitutions.FindPackageShare(package='robot_discription').find('robot_description')

    # Launch lidar sensor
    lidar_launch = IncludeLaunchDescription(
        PathJoinSubstitution(
            [lidar_pkg_share, "launch", "sllidar_a1_launch.py"]))


    # Launch IMU sensor
    imu_launch = IncludeLaunchDescription(
        PathJoinSubstitution(
            [imu_pkg_share, "imu.launch.py"]))

    # Launch robot_state_publisher and localization
    robot_launch = IncludeLaunchDescription(
        PathJoinSubstitution(
            [description_pkg_share, "launch", "robot.launch.py"]))

    # Launch teleop
    teleop_launch = IncludeLaunchDescription(
        PathJoinSubstitution(
            [controller_pkg_share, "launch", "teleop_launch.py"]))

    # Run motor_controller node
    controller_node = Node(
        package='robot_controller',
        executable='motor_controller',
        name='motor_controller',
        output='screen'
    )


    ld = LaunchDescription()
    ld.add_action(lidar_launch)
    ld.add_action(imu_launch)
    ld.add_action(robot_launch)
    ld.add_action(teleop_launch)
    ld.add_action(controller_node)

    return ld


