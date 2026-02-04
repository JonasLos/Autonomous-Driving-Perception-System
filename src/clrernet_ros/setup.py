import os
from glob import glob

from setuptools import setup

package_name = "clrernet_ros"

setup(
    name=package_name,
    version="0.0.0",
    packages=[],
    py_modules=[
        "clrernet_lane_detection",
        "clrernet_lane_transform",
        "inference",
    ],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{package_name}"],
        ),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "clrernet_lane_detection = clrernet_lane_detection:main",
            "clrernet_lane_transform = clrernet_lane_transform:main",
        ],
    },
)
