from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'anthro_arm_mapping'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zhelinyang',
    maintainer_email='jimmy.yang@tum.de',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
          'joints_rt_pub_node = anthro_arm_mapping.joints_rt_pub_node:main',
          'interaction_detect_node = anthro_arm_mapping.interaction_check:main',
          'vicon_udp_to_ros2 = anthro_arm_mapping.vicon_udp_to_ros2:main',
        ],
    },
)
