import os
from time import sleep

import launch
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

launch_path = os.path.realpath(__file__).replace("feature_tracker_launch.py", "")

VINS_Mono_Path = os.path.realpath(
    os.path.join(launch_path, "../../../..","VINS-Mono"))

# VINS_Mono_config_path = os.path.join(VINS_Mono_Path, "config", "tum")

VINS_Mono_config_path = os.path.join(VINS_Mono_Path, "config", "euroc")

def generate_launch_description():
    ld = launch.LaunchDescription()

    namespace = "VINS_ROS2"

    config = os.path.join(
        VINS_Mono_config_path,
        # "bluefox_fisheye_9250.yaml"
        "euroc_config.yaml"
    )


    pose_graph = Node(
        package="pose_graph",
        executable="pose_graph",
        output="screen",
        parameters=[
            {"config_file": config},
            {"visualization_shift_x":0},
            {"visualization_shift_y": 0},
            {"skip_cnt": 0},
            {"skip_dis":0},
            {"vins_path": VINS_Mono_Path}
        ],
        # arguments = ['--ros-args', '--log-level', 'DEBUG']
    )

    # camera_to_cam = Node(package = "tf2_ros",
    #     executable = "static_transform_publisher",
    #     output="screen",
    #     arguments = ["0", "0", "0", "0", "0", "0", "camera", "bluefox3_F0900056"]
    # )


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
    ld.add_action(pose_graph)
    # ld.add_action(world_to_map)

    return ld
