import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'px4_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='px4_control',
    maintainer_email='px4-control@example.com',
    description='MAVROS-style offboard control for PX4 v1.17 over uXRCE-DDS.',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'px4_control_node = px4_control.node:main',
            'out_and_back = px4_control.examples.out_and_back:main',
            'fake_vision_source = px4_control.testing.fake_vision:main',
        ],
    },
)
