from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    navigator_topic = LaunchConfiguration("navigator_topic")
    throttled_navigator_topic = LaunchConfiguration("throttled_navigator_topic")
    navigator_throttle_rate = LaunchConfiguration("navigator_throttle_rate")
    battery_topic = LaunchConfiguration("battery_topic")
    leak_topic = LaunchConfiguration("leak_topic")
    gps_topic = LaunchConfiguration("gps_topic")
    throttled_gps_topic = LaunchConfiguration("throttled_gps_topic")
    gps_throttle_rate = LaunchConfiguration("gps_throttle_rate")
    dvl_twist_topic = LaunchConfiguration("dvl_twist_topic")
    throttled_dvl_twist_topic = LaunchConfiguration("throttled_dvl_twist_topic")
    dvl_twist_throttle_rate = LaunchConfiguration("dvl_twist_throttle_rate")
    dvl_altitude_topic = LaunchConfiguration("dvl_altitude_topic")
    throttled_dvl_altitude_topic = LaunchConfiguration("throttled_dvl_altitude_topic")
    dvl_altitude_throttle_rate = LaunchConfiguration("dvl_altitude_throttle_rate")
    imu_topic = LaunchConfiguration("imu_topic")
    throttled_imu_topic = LaunchConfiguration("throttled_imu_topic")
    imu_throttle_rate = LaunchConfiguration("imu_throttle_rate")
    pressure_topic = LaunchConfiguration("pressure_topic")
    throttled_pressure_topic = LaunchConfiguration("throttled_pressure_topic")
    pressure_throttle_rate = LaunchConfiguration("pressure_throttle_rate")
    namespace = "cirtesub"
    diagnostics_remappings = [
        ("/diagnostics", "/cirtesub/diagnostics"),
        ("/diagnostics_agg", "/cirtesub/diagnostics_agg"),
        ("/diagnostics_toplevel_state", "/cirtesub/diagnostics_toplevel_state"),
    ]

    diagnostics_pkg = get_package_share_directory("sura_diagnostics")
    aggregator_config = os.path.join(
        diagnostics_pkg, "config", "diagnostic_aggregator.yaml"
    )
    controller_debug_config = os.path.join(
        diagnostics_pkg, "config", "controller_debug_diagnostics.yaml"
    )
    battery_status_config = os.path.join(
        diagnostics_pkg, "config", "battery_status_diagnostics.yaml"
    )
    navigation_limits_config = os.path.join(
        diagnostics_pkg, "config", "navigation_limits_diagnostics.yaml"
    )
    gps_diagnostics_config = os.path.join(
        diagnostics_pkg, "config", "gps_diagnostics.yaml"
    )
    dvl_diagnostics_config = os.path.join(
        diagnostics_pkg, "config", "dvl_diagnostics.yaml"
    )
    dvl_altitude_diagnostics_config = os.path.join(
        diagnostics_pkg, "config", "dvl_altitude_diagnostics.yaml"
    )
    imu_diagnostics_config = os.path.join(
        diagnostics_pkg, "config", "imu_diagnostics.yaml"
    )
    pressure_diagnostics_config = os.path.join(
        diagnostics_pkg, "config", "pressure_diagnostics.yaml"
    )
    leak_sensors_config = os.path.join(
        diagnostics_pkg, "config", "leak_sensors_diagnostics.yaml"
    )
    hardware_components_config = os.path.join(
        diagnostics_pkg, "config", "hardware_components_diagnostics.yaml"
    )
    robot_watchdog_config = os.path.join(
        diagnostics_pkg, "config", "robot_watchdog.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "navigator_topic",
                default_value="/cirtesub/navigator/navigation",
            ),
            DeclareLaunchArgument(
                "throttled_navigator_topic",
                default_value="/cirtesub/navigator/navigation_throttled",
            ),
            DeclareLaunchArgument(
                "navigator_throttle_rate",
                default_value="1.0",
            ),
            DeclareLaunchArgument(
                "battery_topic",
                default_value="/cirtesub/battery/status",
            ),
            DeclareLaunchArgument(
                "leak_topic",
                default_value="/cirtesub/leak_sensors",
            ),
            DeclareLaunchArgument(
                "gps_topic",
                default_value="/cirtesub/sensors/gps",
            ),
            DeclareLaunchArgument(
                "throttled_gps_topic",
                default_value="/cirtesub/sensors/gps_throttled",
            ),
            DeclareLaunchArgument(
                "gps_throttle_rate",
                default_value="1.0",
            ),
            DeclareLaunchArgument(
                "dvl_twist_topic",
                default_value="/cirtesub/sensors/dvl/twist",
            ),
            DeclareLaunchArgument(
                "throttled_dvl_twist_topic",
                default_value="/cirtesub/sensors/dvl/twist_throttled",
            ),
            DeclareLaunchArgument(
                "dvl_twist_throttle_rate",
                default_value="1.0",
            ),
            DeclareLaunchArgument(
                "dvl_altitude_topic",
                default_value="/cirtesub/sensors/dvl/altitude",
            ),
            DeclareLaunchArgument(
                "throttled_dvl_altitude_topic",
                default_value="/cirtesub/sensors/dvl/altitude_throttled",
            ),
            DeclareLaunchArgument(
                "dvl_altitude_throttle_rate",
                default_value="1.0",
            ),
            DeclareLaunchArgument(
                "imu_topic",
                default_value="/cirtesub/sensors/imu",
            ),
            DeclareLaunchArgument(
                "throttled_imu_topic",
                default_value="/cirtesub/sensors/imu_throttled",
            ),
            DeclareLaunchArgument(
                "imu_throttle_rate",
                default_value="1.0",
            ),
            DeclareLaunchArgument(
                "pressure_topic",
                default_value="/cirtesub/sensors/pressure",
            ),
            DeclareLaunchArgument(
                "throttled_pressure_topic",
                default_value="/cirtesub/sensors/pressure_throttled",
            ),
            DeclareLaunchArgument(
                "pressure_throttle_rate",
                default_value="1.0",
            ),
            Node(
                package="topic_tools",
                executable="throttle",
                name="navigator_navigation_throttle",
                namespace=namespace,
                output="screen",
                arguments=[
                    "messages",
                    navigator_topic,
                    navigator_throttle_rate,
                    throttled_navigator_topic,
                ],
            ),
            Node(
                package="topic_tools",
                executable="throttle",
                name="gps_throttle",
                namespace=namespace,
                output="screen",
                arguments=[
                    "messages",
                    gps_topic,
                    gps_throttle_rate,
                    throttled_gps_topic,
                ],
            ),
            Node(
                package="topic_tools",
                executable="throttle",
                name="dvl_twist_throttle",
                namespace=namespace,
                output="screen",
                arguments=[
                    "messages",
                    dvl_twist_topic,
                    dvl_twist_throttle_rate,
                    throttled_dvl_twist_topic,
                ],
            ),
            Node(
                package="topic_tools",
                executable="throttle",
                name="dvl_altitude_throttle",
                namespace=namespace,
                output="screen",
                arguments=[
                    "messages",
                    dvl_altitude_topic,
                    dvl_altitude_throttle_rate,
                    throttled_dvl_altitude_topic,
                ],
            ),
            Node(
                package="topic_tools",
                executable="throttle",
                name="imu_throttle",
                namespace=namespace,
                output="screen",
                arguments=[
                    "messages",
                    imu_topic,
                    imu_throttle_rate,
                    throttled_imu_topic,
                ],
            ),
            Node(
                package="topic_tools",
                executable="throttle",
                name="pressure_throttle",
                namespace=namespace,
                output="screen",
                arguments=[
                    "messages",
                    pressure_topic,
                    pressure_throttle_rate,
                    throttled_pressure_topic,
                ],
            ),
            Node(
                package="sura_diagnostics",
                executable="controller_debug_diagnostics",
                namespace=namespace,
                output="screen",
                parameters=[controller_debug_config],
                remappings=diagnostics_remappings,
            ),
            Node(
                package="sura_diagnostics",
                executable="navigation_limits_diagnostics",
                namespace=namespace,
                output="screen",
                parameters=[
                    navigation_limits_config,
                    {
                        "navigator_topic": throttled_navigator_topic,
                        "frequency_topic": navigator_topic,
                    },
                ],
                remappings=diagnostics_remappings,
            ),
            Node(
                package="sura_diagnostics",
                executable="gps_diagnostics",
                namespace=namespace,
                output="screen",
                parameters=[
                    gps_diagnostics_config,
                    {
                        "sensor_topic": throttled_gps_topic,
                        "frequency_topic": gps_topic,
                    },
                ],
                remappings=diagnostics_remappings,
            ),
            Node(
                package="sura_diagnostics",
                executable="dvl_diagnostics",
                namespace=namespace,
                output="screen",
                parameters=[
                    dvl_diagnostics_config,
                    {
                        "sensor_topic": throttled_dvl_twist_topic,
                        "frequency_topic": dvl_twist_topic,
                    },
                ],
                remappings=diagnostics_remappings,
            ),
            Node(
                package="sura_diagnostics",
                executable="dvl_altitude_diagnostics",
                namespace=namespace,
                output="screen",
                parameters=[
                    dvl_altitude_diagnostics_config,
                    {
                        "sensor_topic": throttled_dvl_altitude_topic,
                        "frequency_topic": dvl_altitude_topic,
                    },
                ],
                remappings=diagnostics_remappings,
            ),
            Node(
                package="sura_diagnostics",
                executable="imu_diagnostics",
                namespace=namespace,
                output="screen",
                parameters=[
                    imu_diagnostics_config,
                    {
                        "sensor_topic": throttled_imu_topic,
                        "frequency_topic": imu_topic,
                    },
                ],
                remappings=diagnostics_remappings,
            ),
            Node(
                package="sura_diagnostics",
                executable="pressure_diagnostics",
                namespace=namespace,
                output="screen",
                parameters=[
                    pressure_diagnostics_config,
                    {
                        "sensor_topic": throttled_pressure_topic,
                        "frequency_topic": pressure_topic,
                    },
                ],
                remappings=diagnostics_remappings,
            ),
            Node(
                package="sura_diagnostics",
                executable="battery_status_diagnostics",
                namespace=namespace,
                output="screen",
                parameters=[
                    battery_status_config,
                    {"battery_topic": battery_topic},
                ],
                remappings=diagnostics_remappings,
            ),
            Node(
                package="sura_diagnostics",
                executable="leak_sensors_diagnostics",
                namespace=namespace,
                output="screen",
                parameters=[
                    leak_sensors_config,
                    {"leak_topic": leak_topic},
                ],
                remappings=diagnostics_remappings,
            ),
            Node(
                package="sura_diagnostics",
                executable="hardware_components_diagnostics",
                namespace=namespace,
                output="screen",
                parameters=[hardware_components_config],
                remappings=diagnostics_remappings,
            ),
            Node(
                package="sura_diagnostics",
                executable="robot_watchdog",
                namespace=namespace,
                output="screen",
                parameters=[
                    robot_watchdog_config,
                    {"navigator_topic": throttled_navigator_topic},
                ],
                remappings=diagnostics_remappings,
            ),
            Node(
                package="diagnostic_aggregator",
                executable="aggregator_node",
                namespace=namespace,
                output="screen",
                parameters=[aggregator_config],
                remappings=diagnostics_remappings,
            ),
        ]
    )
