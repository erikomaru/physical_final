import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    share = get_package_share_directory("surry_hills_rover")
    world = os.path.join(share, "worlds", "surry_hills.sdf")
    plugin_path = os.environ.get("LD_LIBRARY_PATH", "")
    gazebo = ExecuteProcess(
        cmd=["ign", "gazebo", "-r", world], output="screen",
        additional_env={"IGN_GAZEBO_SYSTEM_PLUGIN_PATH": plugin_path},
    )
    bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge", output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            "/model/rover/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
            "/model/rover/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
        ],
    )
    return LaunchDescription([
        SetParameter(name="use_sim_time", value=True),
        gazebo,
        bridge,
    ])
