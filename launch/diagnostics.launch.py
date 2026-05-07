import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def namespaced_config(config_file, robot_namespace):
    text = Path(config_file).read_text(encoding="utf-8")
    text = text.replace("/cirtesub/", f"/{robot_namespace}/")
    text = text.replace("cirtesub/", f"{robot_namespace}/")
    text = text.replace("cirtesub_", f"{robot_namespace}_")
    text = text.replace("CIRTESUB", robot_namespace.upper())

    output_file = f"/tmp/sura_diagnostics_{robot_namespace}_{Path(config_file).name}"
    Path(output_file).write_text(text, encoding="utf-8")
    return output_file


def throttle_node(name, robot_namespace, input_topic, rate, output_topic):
    return Node(
        package="topic_tools",
        executable="throttle",
        name=name,
        namespace=robot_namespace,
        output="screen",
        arguments=["messages", input_topic, rate, output_topic],
    )


def diagnostics_node(executable, robot_namespace, config_file, overrides, remappings):
    parameters = [config_file]
    if overrides:
        parameters.append(overrides)

    return Node(
        package="sura_diagnostics",
        executable=executable,
        namespace=robot_namespace,
        output="screen",
        parameters=parameters,
        remappings=remappings,
    )


def launch_setup(context, *args, **kwargs):
    robot_namespace = LaunchConfiguration("robot_namespace").perform(context).strip("/")
    robot_variant = LaunchConfiguration("robot_variant").perform(context)
    diagnostics_pkg = get_package_share_directory("sura_diagnostics")

    def topic(path):
        return f"/{robot_namespace}/{path}"

    configs = {
        name: namespaced_config(
            os.path.join(diagnostics_pkg, "config", f"{name}.yaml"),
            robot_namespace,
        )
        for name in [
            "diagnostic_aggregator",
            "controller_debug_diagnostics",
            "battery_status_diagnostics",
            "navigation_limits_diagnostics",
            "gps_diagnostics",
            "dvl_diagnostics",
            "dvl_altitude_diagnostics",
            "imu_diagnostics",
            "pressure_diagnostics",
            "leak_sensors_diagnostics",
            "hardware_components_diagnostics",
            "robot_watchdog",
        ]
    }

    diagnostics_remappings = [
        ("/diagnostics", topic("diagnostics")),
        ("/diagnostics_agg", topic("diagnostics_agg")),
        ("/diagnostics_toplevel_state", topic("diagnostics_toplevel_state")),
    ]

    navigator_topic = topic("navigator/navigation")
    throttled_navigator_topic = topic("navigator/navigation_throttled")
    gps_topic = topic("sensors/gps")
    throttled_gps_topic = topic("sensors/gps_throttled")
    dvl_twist_topic = topic("sensors/dvl/twist")
    throttled_dvl_twist_topic = topic("sensors/dvl/twist_throttled")
    dvl_altitude_topic = topic("sensors/dvl/altitude")
    throttled_dvl_altitude_topic = topic("sensors/dvl/altitude_throttled")
    imu_topic = topic("sensors/imu")
    throttled_imu_topic = topic("sensors/imu_throttled")
    pressure_topic = topic("sensors/pressure")
    throttled_pressure_topic = topic("sensors/pressure_throttled")

    hardware_component_names = [f"{robot_namespace}_thrusters"]
    if robot_variant == "dual_alpha":
        hardware_component_names.extend(
            [f"{robot_namespace}/alpha_left", f"{robot_namespace}/alpha_right"]
        )

    return [
        throttle_node(
            "navigator_navigation_throttle",
            robot_namespace,
            navigator_topic,
            LaunchConfiguration("navigator_throttle_rate"),
            throttled_navigator_topic,
        ),
        throttle_node(
            "gps_throttle",
            robot_namespace,
            gps_topic,
            LaunchConfiguration("gps_throttle_rate"),
            throttled_gps_topic,
        ),
        throttle_node(
            "dvl_twist_throttle",
            robot_namespace,
            dvl_twist_topic,
            LaunchConfiguration("dvl_twist_throttle_rate"),
            throttled_dvl_twist_topic,
        ),
        throttle_node(
            "dvl_altitude_throttle",
            robot_namespace,
            dvl_altitude_topic,
            LaunchConfiguration("dvl_altitude_throttle_rate"),
            throttled_dvl_altitude_topic,
        ),
        throttle_node(
            "imu_throttle",
            robot_namespace,
            imu_topic,
            LaunchConfiguration("imu_throttle_rate"),
            throttled_imu_topic,
        ),
        throttle_node(
            "pressure_throttle",
            robot_namespace,
            pressure_topic,
            LaunchConfiguration("pressure_throttle_rate"),
            throttled_pressure_topic,
        ),
        diagnostics_node(
            "controller_debug_diagnostics",
            robot_namespace,
            configs["controller_debug_diagnostics"],
            {
                "debug_topic": topic("controller/debug"),
                "controller_manager_node": topic("controller/controller_manager"),
            },
            diagnostics_remappings,
        ),
        diagnostics_node(
            "navigation_limits_diagnostics",
            robot_namespace,
            configs["navigation_limits_diagnostics"],
            {
                "navigator_topic": throttled_navigator_topic,
                "frequency_topic": navigator_topic,
            },
            diagnostics_remappings,
        ),
        diagnostics_node(
            "gps_diagnostics",
            robot_namespace,
            configs["gps_diagnostics"],
            {"sensor_topic": throttled_gps_topic, "frequency_topic": gps_topic},
            diagnostics_remappings,
        ),
        diagnostics_node(
            "dvl_diagnostics",
            robot_namespace,
            configs["dvl_diagnostics"],
            {
                "sensor_topic": throttled_dvl_twist_topic,
                "frequency_topic": dvl_twist_topic,
            },
            diagnostics_remappings,
        ),
        diagnostics_node(
            "dvl_altitude_diagnostics",
            robot_namespace,
            configs["dvl_altitude_diagnostics"],
            {
                "sensor_topic": throttled_dvl_altitude_topic,
                "frequency_topic": dvl_altitude_topic,
            },
            diagnostics_remappings,
        ),
        diagnostics_node(
            "imu_diagnostics",
            robot_namespace,
            configs["imu_diagnostics"],
            {"sensor_topic": throttled_imu_topic, "frequency_topic": imu_topic},
            diagnostics_remappings,
        ),
        diagnostics_node(
            "pressure_diagnostics",
            robot_namespace,
            configs["pressure_diagnostics"],
            {
                "sensor_topic": throttled_pressure_topic,
                "frequency_topic": pressure_topic,
            },
            diagnostics_remappings,
        ),
        diagnostics_node(
            "battery_status_diagnostics",
            robot_namespace,
            configs["battery_status_diagnostics"],
            {"battery_topic": topic("battery/status")},
            diagnostics_remappings,
        ),
        diagnostics_node(
            "leak_sensors_diagnostics",
            robot_namespace,
            configs["leak_sensors_diagnostics"],
            {"leak_topic": topic("leak_sensors")},
            diagnostics_remappings,
        ),
        diagnostics_node(
            "hardware_components_diagnostics",
            robot_namespace,
            configs["hardware_components_diagnostics"],
            {
                "list_hardware_components_service": topic(
                    "controller/controller_manager/list_hardware_components"
                ),
                "component_names": hardware_component_names,
            },
            diagnostics_remappings,
        ),
        diagnostics_node(
            "robot_watchdog",
            robot_namespace,
            configs["robot_watchdog"],
            {"navigator_topic": throttled_navigator_topic},
            diagnostics_remappings,
        ),
        Node(
            package="diagnostic_aggregator",
            executable="aggregator_node",
            namespace=robot_namespace,
            output="screen",
            parameters=[configs["diagnostic_aggregator"]],
            remappings=diagnostics_remappings,
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_namespace", default_value="sura"),
            DeclareLaunchArgument("robot_variant", default_value="dual_alpha"),
            DeclareLaunchArgument("navigator_throttle_rate", default_value="1.0"),
            DeclareLaunchArgument("gps_throttle_rate", default_value="1.0"),
            DeclareLaunchArgument("dvl_twist_throttle_rate", default_value="1.0"),
            DeclareLaunchArgument("dvl_altitude_throttle_rate", default_value="1.0"),
            DeclareLaunchArgument("imu_throttle_rate", default_value="1.0"),
            DeclareLaunchArgument("pressure_throttle_rate", default_value="1.0"),
            OpaqueFunction(function=launch_setup),
        ]
    )
