from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'amr_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ikram',
    maintainer_email='ikramabdiazizyusuf@gmail.com',
    description='Autonomous Mobile Robot (AMR) Navigation with SLAM, EKF, custom A* planner and dynamic obstacle avoidance',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'astar_planner_node = amr_navigation.astar_planner_node:main',
            'dynamic_avoidance_node = amr_navigation.dynamic_avoidance_node:main',
        ],
    },
)
