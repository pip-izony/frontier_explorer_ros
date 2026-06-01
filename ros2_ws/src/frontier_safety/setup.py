from setuptools import setup
import os
from glob import glob

package_name = 'frontier_safety'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Reactive LiDAR collision avoidance for the sjtu_drone frontier explorer.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'collision_avoidance = frontier_safety.collision_avoidance:main',
        ],
    },
)
