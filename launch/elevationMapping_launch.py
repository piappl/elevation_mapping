import os

import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share_dir = get_package_share_directory("elevation_mapping")
    config_dir = os.path.join(share_dir, "config")
    xacro_path = os.path.join(config_dir, "robot.urdf.xacro")
    list_params = []
    for filee in [
        "robots/piap_hunter.yaml",
        # "elevation_maps/long_range.yaml",
        "elevation_maps/default.yaml",
        "sensor_processors/Livox_mid360.yaml",
        # "postprocessing/postprocessor_pipeline.yaml",
    ]:
        list_params.append(os.path.join(config_dir, filee))

    namespace_arg = DeclareLaunchArgument(
        "namespace", default_value="piap_hunter", description="Node namespace"
    )
    namespace = LaunchConfiguration("namespace")

    use_sim_time = LaunchConfiguration("use_sim_time")
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )
    list_params.append(
        {"use_sim_time": use_sim_time, "ros__parameters": {"ros_log_level": "debug"}}
    )

    return launch.LaunchDescription(
        [
            declare_use_sim_time_cmd,
            launch_ros.actions.Node(
                package="elevation_mapping",
                executable="elevation_mapping",
                name="elevation_mapping",
                output="screen",
                # arguments=["--ros-args", "--log-level", "debug"],
                parameters=list_params,
                # namespace=namespace,
                # remappings=[
                #     ("/tf", "/piap_hunter/tf"),
                #     ("/tf_static", "/piap_hunter/tf_static"),
                # ],
            ),
            # launch_ros.actions.Node(
            # package='robot_state_publisher',
            # executable='robot_state_publisher',
            # name='robot_state_publisher',
            # output='screen',
            # parameters=[{
            #    'robot_description': launch.substitutions.Command(['xacro', ' ', xacro_path])
            # }]
            # )
        ]
    )
