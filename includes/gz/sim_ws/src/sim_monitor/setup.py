from setuptools import setup

package_name = "sim_monitor"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    description="Gazebo real-time factor, spawn TF, and preflight checks.",
    entry_points={
        "console_scripts": [
            "sim_real_time_factor = sim_monitor.real_time_factor:main",
            "sim_spawn_frame = sim_monitor.spawn_frame:main",
            "sim_preflight_check = sim_monitor.preflight_check:main",
        ],
    },
)
