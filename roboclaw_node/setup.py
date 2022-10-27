## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD
import os
from glob import glob
from setuptools import setup

package_name = 'roboclaw_node_ros'

# fetch values from package.xml
setup(
    name='roboclaw_node_ros',
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
        # Include all config files
        (os.path.join('share', package_name), glob('config/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Brad Bazemore',
    maintainer_email='bwbazemore@uga.edu',
    description='Node for roboclaw',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'roboclaw_node = roboclaw_node_ros.roboclaw_node:main',
        ],
    },
)