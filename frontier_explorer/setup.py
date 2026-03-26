from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'frontier_explorer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lucia Ferreira',
    maintainer_email='ana.lucia.piferreira@gmail.com',
    description='A ROS 2 package implementing frontier-based autonomous exploration with Nav2, including obstacle-aware goal selection, stuck recovery, and automatic map saving.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'explorer = frontier_explorer.navigation_explorer:main',
        ],
    },
)
