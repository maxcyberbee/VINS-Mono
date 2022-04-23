import os
from time import sleep

import launch
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

launch_path = os.path.realpath(__file__).replace("feature_tracker_launch_max.py", "")

VINS_Mono_path = os.path.realpath(
    os.path.join(launch_path, "../../../../..","src","VINS-Mono"))

VINS_Mono_config_path = os.path.join(VINS_Mono_path, "config", "euroc")


def generate_launch_description():
    ld = launch.LaunchDescription()

    namespace = "VINS_ROS2"

    config = os.path.join(
        VINS_Mono_config_path,
        "euroc_config.yaml"
    )

    rosbag = launch.actions.ExecuteProcess(
        cmd=['ros2', 'bag', 'play', os.path.join(VINS_Mono_path, 'bags', 'V1_01_easy', 'V1_01_easy.db3')],
        output='screen'
    )

    rviz = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', [os.path.join(VINS_Mono_path, 'config', 'vins_rviz2_config.rviz')]]
    )


    # rosbag = Node(
    #     package='bag',
    #     executable='play',
    #     # node_name='rosbag2_node',
    #     arguments=[],
    #     parameters=[],
    #     output='screen')

    feature_tracker_node = Node(
        package="feature_tracker",
        executable="feature_tracker",
        output="screen",
        parameters=[
            {"config_file": config}
        ]

    )

    vins_estimator = Node(
        package="vins_estimator",
        executable="vins_estimator",
        output="screen",
        parameters=[
            {"config_file": config}
        ]
    )

    camera_to_cam = Node(package = "tf2_ros",
                executable = "static_transform_publisher",
                output="screen",
                arguments = ["0", "0", "0", "0", "0", "0", "camera", "cam0"]
                )
    world_to_map = Node(package = "tf2_ros",
                         executable = "static_transform_publisher",
                         output="screen",
                         arguments = ["0", "0", "0", "0", "0", "0", "world", "map"]
                         )
    # trajectory_controller_node = Node(
    #     package="offboard_exp",
    #     namespace=namespace,
    #     executable="trajectory_controller",
    #     name="trajectory_controller_{:s}".format(namespace)
    # )

    # ld.add_action(trajectory_controller_node)
    ld.add_action(rviz)
    ld.add_action(rosbag)
    ld.add_action(feature_tracker_node)
    ld.add_action(vins_estimator)
    # ld.add_action(camera_to_cam)
    # ld.add_action(world_to_map)

    return ld