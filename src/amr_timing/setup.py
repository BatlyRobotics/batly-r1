from setuptools import find_packages, setup
from glob import glob
import os

package_name = "amr_timing"

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
    entry_points={
        "console_scripts": [
            "timing_node = amr_timing.timing_node:main",
        ],
    },
)
