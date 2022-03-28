import os
from time import sleep


from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory


launch_path = os.path.realpath(__file__).replace("feature_tracker_launch.py", "")
ros2_ws = os.path.realpath(os.path.relpath(
    os.path.join(launch_path, "../../../../..")))

VINS_Mono_path = os.path.join(ros2_ws, "src", "VINS-Mono")
VINS_Mono_config_path = os.path.join(VINS_Mono_path, "config","euroc")


def generate_launch_description():

    ld = LaunchDescription()

    namespace = "VINS_ROS2"

    config = os.path.join(
        VINS_Mono_config_path,        
        "euroc_config.yaml"
    )

    feature_tracker_node = Node(
        package="feature_tracker",        
        executable="feature_tracker",
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

    #ld.add_action(trajectory_controller_node)
    ld.add_action(feature_tracker_node)

    return ld
