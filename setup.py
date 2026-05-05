from setuptools import setup
import os
from glob import glob


package_name = "sura_diagnostics"


setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="usuario",
    maintainer_email="iedo@uji.es",
    description="SURA diagnostics and control observability tools",
    license="TODO: License declaration",
    entry_points={
        "console_scripts": [
            "battery_status_diagnostics = sura_diagnostics.battery_status_diagnostics:main",
            "controller_debug_diagnostics = sura_diagnostics.controller_debug_diagnostics:main",
            "dvl_altitude_diagnostics = sura_diagnostics.dvl_altitude_diagnostics:main",
            "dvl_diagnostics = sura_diagnostics.dvl_diagnostics:main",
            "gps_diagnostics = sura_diagnostics.gps_diagnostics:main",
            "hardware_components_diagnostics = sura_diagnostics.hardware_components_diagnostics:main",
            "imu_diagnostics = sura_diagnostics.imu_diagnostics:main",
            "leak_sensors_diagnostics = sura_diagnostics.leak_sensors_diagnostics:main",
            "navigation_limits_diagnostics = sura_diagnostics.navigation_limits_diagnostics:main",
            "pressure_diagnostics = sura_diagnostics.pressure_diagnostics:main",
            "robot_watchdog = sura_diagnostics.robot_watchdog:main",
        ],
    },
)
