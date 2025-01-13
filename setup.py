# ! DO NOT MANUALLY INVOKE THIS setup.py, USE COLCON INSTEAD
import os
from glob import glob
from setuptools import setup

package_name = "roboclaw_ros"

# fetch values from package.xml
setup(
    name="roboclaw_ros",
    version="0.0.2",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Include all launch files.
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*launch.[pxy][yma]*"),
        ),
        # Include all config files
        (os.path.join("share", package_name, "config"), glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    author="Brad Bazemore",
    author_email="bwbazemore@uga.edu",
    maintainer="norlab-marmotte",
    maintainer_email="norlabmarmotte@gmail.com",
    description="RoboClaw Driver for ROS 2",
    license="BSD",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "roboclaw_dual = roboclaw_ros.roboclaw_dual_node:main",
            "roboclaw_solo = roboclaw_ros.roboclaw_solo_node:main",
        ],
    },
)
