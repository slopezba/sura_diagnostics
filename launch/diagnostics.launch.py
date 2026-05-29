import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def load_profile(context):
    package = LaunchConfiguration("diagnostics_params_package").perform(context).strip()
    relative_file = LaunchConfiguration("diagnostics_params_file").perform(context).strip()
    profile_path = os.path.join(get_package_share_directory(package), relative_file)
    with open(profile_path, "r", encoding="utf-8") as file:
        return yaml.safe_load(file) or {}


def topic(robot_namespace, value):
    value = str(value).strip()
    if not value:
        return ""
    if value.startswith("/"):
        return value
    return f"/{robot_namespace}/{value.strip('/')}"


def enabled(config):
    return bool(config.get("enabled", True))


def format_value(value, robot_namespace):
    if isinstance(value, str):
        return value.format(namespace=robot_namespace)
    if isinstance(value, list):
        return [format_value(item, robot_namespace) for item in value]
    if isinstance(value, dict):
        return {
            key: format_value(item, robot_namespace)
            for key, item in value.items()
        }
    return value


def resolve_parameter_values(parameters, robot_namespace):
    resolved = {}
    for key, value in (parameters or {}).items():
        value = format_value(value, robot_namespace)
        if (
            key.endswith("_topic")
            or key == "topic"
            or key.endswith("_service")
            or key.endswith("_node")
        ):
            value = topic(robot_namespace, value)
        resolved[key] = value
    return resolved


def diagnostics_node(executable, node_name, parameters):
    return Node(
        package="sura_diagnostics",
        executable=executable,
        name=node_name,
        output="screen",
        parameters=[parameters],
    )


def throttle_node(name, input_topic, rate, output_topic):
    return Node(
        package="topic_tools",
        executable="throttle",
        name=name,
        output="screen",
        arguments=["messages", input_topic, str(rate), output_topic],
    )


def aggregator_node(profile, robot_namespace):
    config = profile.get("aggregator", {})
    parameters = resolve_parameter_values(config.get("parameters", {}), robot_namespace)
    if not parameters.get("path"):
        parameters["path"] = robot_namespace.upper()

    return Node(
        package="diagnostic_aggregator",
        executable="aggregator_node",
        name="analyzers",
        output="screen",
        parameters=[parameters],
    )


def add_simple_diagnostic(actions, name, config, robot_namespace):
    if not enabled(config):
        return

    executable = config.get("executable", name)
    node_name = config.get("node_name", executable)
    parameters = resolve_parameter_values(
        config.get("parameters", {}),
        robot_namespace,
    )
    actions.append(diagnostics_node(executable, node_name, parameters))


def add_sensor_diagnostic(actions, name, config, robot_namespace):
    if not enabled(config):
        return

    executable = str(config.get("executable", "")).strip()
    if not executable:
        raise RuntimeError(f"Diagnostic sensor '{name}' must define an executable.")
    node_name = config.get("node_name", executable)

    raw_topic = topic(robot_namespace, config.get("topic", ""))
    if not raw_topic:
        raise RuntimeError(f"Diagnostic sensor '{name}' must define a topic.")

    throttle_rate = config.get("throttle_rate", 1.0)
    throttled_suffix = config.get("throttled_topic", "")
    throttled_topic = topic(robot_namespace, throttled_suffix)

    sensor_topic = raw_topic
    if throttle_rate and throttled_topic:
        actions.append(throttle_node(f"{name}_throttle", raw_topic, throttle_rate, throttled_topic))
        sensor_topic = throttled_topic

    parameters = resolve_parameter_values(
        config.get("parameters", {}),
        robot_namespace,
    )
    parameters.setdefault("sensor_topic", sensor_topic)
    parameters.setdefault("frequency_topic", raw_topic)
    parameters.setdefault("hardware_id", f"{robot_namespace}_{name}")

    actions.append(diagnostics_node(executable, node_name, parameters))


def launch_setup(context, *args, **kwargs):
    robot_namespace = LaunchConfiguration("robot_namespace").perform(context).strip("/")
    profile = load_profile(context)

    if not robot_namespace:
        raise RuntimeError("Launch argument 'robot_namespace' cannot be empty.")

    actions = []

    for name, config in (profile.get("diagnostics", {}) or {}).items():
        add_simple_diagnostic(actions, name, config or {}, robot_namespace)

    for name, config in (profile.get("sensors", {}) or {}).items():
        add_sensor_diagnostic(actions, name, config or {}, robot_namespace)

    if enabled(profile.get("aggregator", {})):
        actions.append(aggregator_node(profile, robot_namespace))

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_namespace", default_value="sura"),
            DeclareLaunchArgument("diagnostics_params_package", default_value="sura_diagnostics"),
            DeclareLaunchArgument("diagnostics_params_file", default_value="config/diagnostics.yaml"),
            OpaqueFunction(function=launch_setup),
        ]
    )
