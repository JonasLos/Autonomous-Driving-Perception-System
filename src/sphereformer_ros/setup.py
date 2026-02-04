import os
from glob import glob

from setuptools import setup

package_name = "sphereformer_ros"


def _walk_data_files(base_dir: str, install_root: str):
    collected = []
    if not os.path.isdir(base_dir):
        return collected

    for root, _, files in os.walk(base_dir):
        if not files:
            continue
        rel_dir = os.path.relpath(root, base_dir)
        install_dir = os.path.join(install_root, os.path.basename(base_dir), rel_dir)
        file_paths = []
        for f in files:
            src_path = os.path.join(root, f)
            # Guard against broken symlinks or non-regular files that setuptools can't copy.
            if os.path.isfile(src_path):
                file_paths.append(src_path)

        if file_paths:
            collected.append((install_dir, file_paths))
    return collected


data_files = [
    (
        "share/ament_index/resource_index/packages",
        [f"resource/{package_name}"],
    ),
    (f"share/{package_name}", ["package.xml"]),
    (os.path.join("share", package_name, "launch"), glob("launch/*")),
]

data_files += _walk_data_files(
    os.path.join(os.path.dirname(__file__), "src", "SphereFormer"),
    os.path.join("share", package_name),
)
data_files += _walk_data_files(
    os.path.join(os.path.dirname(__file__), "src", "SphereFormer_changes"),
    os.path.join("share", package_name),
)


setup(
    name=package_name,
    version="0.0.0",
    packages=[],
    py_modules=["sphereformer_ros", "semantic_kitti_ros"],
    data_files=data_files,
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "sphereformer_ros = sphereformer_ros:main",
        ],
    },
)
