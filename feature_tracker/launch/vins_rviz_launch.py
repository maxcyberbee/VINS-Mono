import os
from time import sleep

import launch
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

launch_path = os.path.realpath(__file__).replace("feature_tracker_launch.py", "")

VINS_Mono_Path = os.path.realpath(
    os.path.join(launch_path, "../../../../","src","VINS-Mono"))

# VINS_Mono_config_path = os.path.join(VINS_Mono_Path, "config", "tum")

VINS_Mono_config_path = os.path.join(VINS_Mono_Path, "config", "euroc")

def generate_launch_description():
    ld = launch.LaunchDescription()

    namespace = "VINS_ROS2"

    rviz = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', [os.path.join(VINS_Mono_Path, 'rviz', 'default.rviz')]]
    )


    # camera_to_cam = Node(package = "tf2_ros",
    #     executable = "static_transform_publisher",
    #     output="screen",
    #     arguments = ["0", "0", "0", "0", "0", "0", "camera", "bluefox3_F0900056"]
    # )

    # arguments = ['--ros-args', '--log-level', 'DEBUG']
    
    # world_to_map = Node(package = "tf2_ros",
    #                      executable = "static_transform_publisher",
    #                      output="screen",
    #                      arguments = ["0", "0", "0", "0", "0", "0", "world", "map"]
    #                      )
    # # trajectory_controller_node = Node(
    # #     package="offboard_exp",
    # #     namespace=namespace,
    # #     executable="trajectory_controller",
    # #     name="trajectory_controller_{:s}".format(namespace)
    # # )

    # # ld.add_action(trajectory_controller_node)
    ld.add_action(rviz)

    # ld.add_action(world_to_map)

    return ld
