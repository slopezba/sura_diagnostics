# SURA Diagnostics Architecture

This document defines the target structure for `sura_diagnostics`. The goal is to keep the package focused: it observes the robot, publishes standard ROS 2 diagnostics, and produces safety-relevant information that higher-level autonomy can consume later.

## Purpose

`sura_diagnostics` is responsible for answering:

- Is the robot publishing the required state?
- Are key sensors alive and fresh?
- Is the computer healthy enough to continue?
- Are safety-critical signals nominal?
- What diagnostics should be visible through standard ROS 2 tooling?

It should not execute missions, change controllers, or decide the full autonomy behavior. Those decisions belong to the future Behavior Tree package.

## Package Boundaries

### `sura_diagnostics`

Owns:

- Watchdogs for topics and sensors.
- Frequency checks.
- Low-rate computer resource checks.
- Safety diagnostics such as leak, battery, navigation freshness, localization freshness, and controller health.
- Publishing `diagnostic_msgs/msg/DiagnosticArray` on `/diagnostics`.
- Grouping diagnostics through `diagnostic_aggregator`.

Does not own:

- Mission execution.
- Behavior Tree logic.
- Controller activation/deactivation.
- Navigation goals.
- Emergency behavior execution.

### `sura_msgs`

Owns shared SURA interfaces when needed.

For the first diagnostic steps, we should avoid adding new custom messages unless standard ROS 2 diagnostics are not enough.

### Future `sura_behavior`

Will consume diagnostics or an aggregated SURA safety state and decide what to execute:

- Hold position.
- Return home.
- Surface.
- Abort autonomy.
- Run autonomous mission subtree.

## Current First Step

The first safety diagnostic is:

```text
Navigator topic frequency monitor
```

It subscribes to the navigator topic:

```text
/cirtesub/navigator/navigation
```

and publishes a standard diagnostic:

```text
/diagnostics
  /Navigation/Frequency
```

This diagnostic is evaluated every 2 seconds by default.

## Target Directory Structure

```text
sura_diagnostics/
├── ARCHITECTURE.md
├── config/
│   ├── diagnostic_aggregator.yaml
│   ├── navigator_frequency_diagnostics.yaml
│   ├── system_diagnostics.yaml
│   ├── battery_diagnostics.yaml
│   ├── leak_diagnostics.yaml
│   ├── sensor_watchdog_diagnostics.yaml
│   └── localization_frequency_diagnostics.yaml
├── launch/
│   └── diagnostics.launch.py
├── sura_diagnostics/
│   ├── controller_debug_diagnostics.py
│   ├── navigator_frequency_diagnostics.py
│   ├── system_diagnostics.py
│   ├── battery_diagnostics.py
│   ├── leak_diagnostics.py
│   ├── sensor_watchdog_diagnostics.py
│   └── localization_frequency_diagnostics.py
├── package.xml
└── setup.py
```

Not all files should be implemented immediately. This is the target shape.

## Diagnostic Topics

All diagnostic producers publish to:

```text
/diagnostics
```

using:

```text
diagnostic_msgs/msg/DiagnosticArray
diagnostic_msgs/msg/DiagnosticStatus
```

The `diagnostic_aggregator` groups them into readable categories.

## Diagnostic Names

Use stable names because rules, dashboards, and future BT conditions may depend on them.

```text
/Navigation/Frequency
/Localization/Frequency
/Controllers/Summary
/Controllers/<controller_name>
/System/CPU
/System/Memory
/Battery/Status
/LeakSensor/Status
/Sensors/<sensor_name>
```

## Diagnostic Levels

Use standard ROS 2 diagnostic levels:

```text
OK
WARN
ERROR
STALE
```

Suggested meaning:

```text
OK     The signal is healthy.
WARN   Something is degraded but not immediately critical.
ERROR  The robot should not continue autonomous behavior normally.
STALE  Required information is missing or has stopped updating.
```

## Launch Design

The diagnostics launch should start the common monitors and the aggregator.

Initial launch:

```text
controller_debug_diagnostics
navigator_frequency_diagnostics
battery_status_diagnostics
diagnostic_aggregator
```

Later launch:

```text
controller_debug_diagnostics
navigator_frequency_diagnostics
localization_frequency_diagnostics
sensor_watchdog_diagnostics
system_diagnostics
battery_diagnostics
leak_sensors_diagnostics
diagnostic_aggregator
```

Each monitor should be optional through launch arguments once the package grows.

## Frequency Policy

Diagnostics should be low frequency unless the signal is safety-critical.

Suggested defaults:

```text
Navigation frequency monitor:      0.5 Hz  (every 2 seconds)
Localization frequency monitor:    0.5 Hz  (every 2 seconds)
CPU and RAM monitor:               0.5 Hz  (every 2 seconds)
Battery monitor:                   1.0 Hz
Leak detector:                     2.0 Hz or hardware-driven
Sensor watchdog summary:           0.5 Hz to 1.0 Hz
Controller debug diagnostics:      1.0 Hz
```

Avoid publishing CPU, RAM, battery, or summary diagnostics at high frequency.

## Navigator Frequency Diagnostic

Node:

```text
navigator_frequency_diagnostics
```

Input:

```text
sura_msgs/msg/Navigator
```

Default topic:

```text
/cirtesub/navigator/navigation
```

Output:

```text
/diagnostics
```

Diagnostic status name:

```text
/Navigation/Frequency
```

Default parameters:

```yaml
navigator_topic: /cirtesub/navigator/navigation
publish_period: 2.0
warn_min_frequency_hz: 10.0
error_min_frequency_hz: 2.0
stale_timeout: 3.0
```

Expected behavior:

```text
OK     frequency >= warn_min_frequency_hz
WARN   error_min_frequency_hz <= frequency < warn_min_frequency_hz
ERROR  frequency < error_min_frequency_hz
STALE  no message received or last message older than stale_timeout
```

## System Diagnostics

This should come after the navigation frequency monitor is validated.

The first version should only publish:

```text
/System/CPU
/System/Memory
```

Do not read computer temperature in the first version.

Default period:

```text
2 seconds
```

Suggested values:

```yaml
publish_period: 2.0
cpu_warn_percent: 80.0
cpu_error_percent: 95.0
memory_warn_used_percent: 80.0
memory_error_used_percent: 90.0
```

## Future Safety Signals

Add these one by one after the navigation frequency check works:

1. CPU and RAM usage.
2. Battery level.
3. Leak sensors.
4. Sensor watchdogs.
5. Localization frequency.
6. Geofence status.
7. Attitude limits.
8. Depth limits for underwater robots.

Each new diagnostic should be tested independently before adding another one.

## Relation With Behavior Tree

The future BT should not subscribe to every raw sensor topic.

Preferred flow:

```text
robot topics
  -> sura_diagnostics
  -> /diagnostics
  -> optional SURA safety aggregator
  -> sura_behavior
```

At first, the BT can consume a small aggregated safety state. Later we can decide whether that state should be:

```text
diagnostic_msgs/msg/DiagnosticArray
```

or a custom message in:

```text
sura_msgs
```

The custom message should only be added once the diagnostic rules are stable.

## Development Rule

Develop diagnostics in small vertical slices:

1. Add one monitor.
2. Publish to `/diagnostics`.
3. Group it in `diagnostic_aggregator.yaml`.
4. Launch it.
5. Test the OK case.
6. Test WARN, ERROR, and STALE cases.
7. Only then add the next monitor.

This keeps the final safety system understandable and prevents the package from growing in too many directions at once.

## Battery Status Diagnostic

The simulation-side battery source lives in:

```text
cirtesub_stonefish
```

Node:

```text
battery_status_simulated
```

Output:

```text
/cirtesub/battery/status
```

Message:

```text
sensor_msgs/msg/BatteryState
```

The simulated node publishes only the fields needed for the first diagnostic pass:

```text
voltage
current
present
power_supply_status
power_supply_health
power_supply_technology
```

The nominal voltage is configurable and defaults to:

```text
26.0 V
```

The current estimate is intentionally approximate:

```text
Jetson current + base electronics current + thruster command estimate
```

The diagnostic-side monitor lives in:

```text
sura_diagnostics
```

Node:

```text
battery_status_diagnostics
```

Input:

```text
/cirtesub/battery/status
```

Output:

```text
/diagnostics
  /Battery/Status
```

Default behavior:

```text
OK     battery topic fresh, voltage above warning threshold, current below warning threshold
WARN   voltage low or current draw high
ERROR  voltage critical, current draw critical, or battery not present
STALE  battery topic missing or stale
```

## Leak Sensors Diagnostic

The simulation-side leak source lives in:

```text
cirtesub_stonefish
```

Node:

```text
leak_sensors_simulated
```

Output:

```text
/cirtesub/leak_sensors
```

Message:

```text
sura_msgs/msg/LeakSensor
```

The message carries:

```text
header
frame_id
leak_detected
```

The simulated node supports multiple leak sensors through YAML:

```yaml
sensor_frames:
  - cirtesub/main_cylinder
  - cirtesub/battery_cylinder
leak_detected:
  - false
  - false
```

The diagnostic-side monitor lives in:

```text
sura_diagnostics
```

Node:

```text
leak_sensors_diagnostics
```

Input:

```text
/cirtesub/leak_sensors
```

Output:

```text
/diagnostics
  /LeakSensor/Status
```

Default behavior:

```text
OK     all configured leak sensors are fresh and false
ERROR  any configured leak sensor is true
STALE  any configured leak sensor is missing or stale
```
