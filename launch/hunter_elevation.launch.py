from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Default config
    robot_config_path = PathJoinSubstitution(
        [
            FindPackageShare("elevation_mapping"),
            "config",
            "robots",
            "piap_hunter.yaml",
        ]
    )
    elevation_maps_config_path = PathJoinSubstitution(
        [
            FindPackageShare("elevation_mapping"),
            "config",
            "elevation_maps",
            "default.yaml",
        ]
    )
    sensor_processors_config_path = PathJoinSubstitution(
        [
            FindPackageShare("elevation_mapping"),
            "config",
            "sensor_processors",
            "Livox_mid360.yaml",
        ]
    )
    postprocessing_config_path = PathJoinSubstitution(
        [
            FindPackageShare("elevation_mapping"),
            "config",
            "postprocessing",
            "postprocessor_pipeline.yaml",
        ]
    )

    # Namespace
    namespace_arg = DeclareLaunchArgument(
        "namespace", default_value="piap_hunter", description="Node namespace"
    )
    namespace = LaunchConfiguration("namespace")

    # Sub namespace
    sub_namespace_arg = DeclareLaunchArgument(
        "sub_namespace", default_value="", description="Node sub_namespace"
    )
    sub_namespace = LaunchConfiguration("sub_namespace")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )
    use_sim_time = LaunchConfiguration("use_sim_time")

    elevation_node = Node(
        package="elevation_mapping",
        executable="elevation_mapping",
        name="elevation_mapping",
        output="screen",
        parameters=[
            robot_config_path,
            elevation_maps_config_path,
            sensor_processors_config_path,
            postprocessing_config_path,
            {"use_sim_time": use_sim_time},
        ],
        namespace=namespace,
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    return LaunchDescription(
        [
            namespace_arg,
            sub_namespace_arg,
            use_sim_time_arg,
            GroupAction(
                actions=[
                    PushRosNamespace(namespace),
                    PushRosNamespace(sub_namespace),
                    elevation_node,
                ]
            ),
        ]
    )
