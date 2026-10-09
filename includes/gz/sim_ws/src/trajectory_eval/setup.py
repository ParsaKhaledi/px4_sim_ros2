from setuptools import setup

package_name = "trajectory_eval"

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
    description="Compare simulation trajectories against Gazebo ground truth.",
    entry_points={
        "console_scripts": [
            "trajectory_eval = trajectory_eval.cli:main",
        ],
    },
)
