from setuptools import find_packages, setup
from glob import glob
import os

package_name = "batly_bringup"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "launch"),
         glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer='Batly R1 Team',
    maintainer_email='pisak.ch@eng.buu.ac.th',
    description="Python ROS 2 driver for ZLAC8015D RS485 Modbus",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "zlac_node = batly_bringup.zlac_node:main",
            "teleop_keyboard = batly_bringup.teleop_keyboard:main",
        ],
    },
)
