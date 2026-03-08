from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'elephant_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*'))
    ],
    install_requires=['setuptools',
                      'rclpy',      # Essential for ROS2 Python
                      'std_msgs',   # Essential for String messages
                      'rcl_interfaces',
    ],
    zip_safe=True,
    maintainer='chaiyapruk',
    maintainer_email='chaiyapruk6494@gmail.com',
    description='control Elephant AGV via camera',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'teleop_honomic = elephant_control.teleop_holonomic:main',
            'cam_teleop = elephant_control.cam_teleop:main',
            'cam_publisher = elephant_control.cam_publisher:main',
        ],
    },
)
