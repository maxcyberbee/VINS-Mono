import os
from time import sleep

import launch
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

launch_path = os.path.realpath(__file__).replace("pose_graph_launch.py", "")

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

    # rosbag = launch.actions.ExecuteProcess(
    #     cmd=['ros2', 'bag', 'play', os.path.join(VINS_Mono_path, 'bags', 'V1_01_easy', 'V1_01_easy.db3')],
    #     output='screen'
    # )

    rviz = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', [os.path.join(VINS_Mono_path,  "config", "vins_rviz2_config.rviz")]]
    )


    # rosbag = Node(
    #     package='bag',
    #     executable='play',
    #     # node_name='rosbag2_node',
    #     arguments=[],
    #     parameters=[],
    #     output='screen')
    
    # feature_tracker_node = Node(
    #     package="feature_tracker",
    #     executable="feature_tracker",
    #     output="screen",
    #     parameters=[
    #         {"config_file": config}
    #     ]

    # )
    
    pose_graph_node = Node(
        package="pose_graph",
        executable="pose_graph",
        output="screen",
        parameters=[
            {"config_file": config}
        ]

    )

    # trajectory_controller_node = Node(
    #     package="offboard_exp",
    #     namespace=namespace,
    #     executable="trajectory_controller",
    #     name="trajectory_controller_{:s}".format(namespace)
    # )

    # ld.add_action(trajectory_controller_node)
    # ld.add_action(rviz)
    # ld.add_action(rosbag)
    # ld.add_action(feature_tracker_node)
    ld.add_action(pose_graph_node)

    return ld
