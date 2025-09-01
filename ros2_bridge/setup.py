from setuptools import setup

package_name = 'ros2_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'yaw_node = ros2_bridge.yaw_node:main',
            'speed_node = ros2_bridge.speed_node:main',
            'steer_node = ros2_bridge.steer_node:main',
        ],
    },
)
