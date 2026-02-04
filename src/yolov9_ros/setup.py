import os
from glob import glob

from setuptools import setup

package_name = "yolov9_ros"


def _maybe(path: str) -> list[str]:
    return [path] if os.path.exists(path) else []


setup(
    name=package_name,
    version="0.0.0",
    packages=[],
    py_modules=[
        "yolov9_object_detection",
        "objects_transform",
    ],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{package_name}"],
        ),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*")),
        (f"share/{package_name}", glob("src/*.yaml")),
        (f"share/{package_name}", _maybe("best.pt")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    author="avalocal",
    description="YOLOv9 ROS2 package (ament_python)",
    classifiers=["Programming Language :: Python :: 3"],
    entry_points={
        "console_scripts": [
            "yolov9_object_detection = yolov9_object_detection:main",
            "objects_transform = objects_transform:main",
        ],
    },
)
