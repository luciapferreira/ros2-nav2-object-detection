from setuptools import setup
import os
from glob import glob

package_name = 'robot_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'map'), glob('map/*')),
        (os.path.join('share', package_name, 'models'), glob('models/*')),
        (os.path.join('share', package_name, 'world'), glob('world/*'))


    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lucia Ferreira',
    maintainer_email='ana.lucia.piferreira@gmail.com',
    description='A ROS 2 package for autonomous robot navigation using Nav2, SLAM, AMCL, YOLOv8 object detection, and a custom navigation goal service in a Gazebo Classic simulation.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'navigate_server = robot_navigation.navigate_server:main',
            'navigate_client = robot_navigation.navigate_client:main',
            'object_detection = robot_navigation.object_detection:main'
        ],
    },
)
