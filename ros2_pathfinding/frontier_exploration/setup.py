from setuptools import setup
import os
from glob import glob

package_name = 'frontier_exploration'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'maps'), []),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Team 20',
    maintainer_email='example@example.com',
    description='Frontier exploration package for autonomous navigation',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'frontier_exploration = frontier_exploration.frontier_exploration:main',
            'pure_pursuit = frontier_exploration.pure_pursuit:main',
            'map_sync = frontier_exploration.map_sync:main',
        ],
    },
) 