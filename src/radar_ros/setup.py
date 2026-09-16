from setuptools import setup

package_name = "radar_ros"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{package_name}"],
        ),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/radar_fusion.launch.py"]),
        (f"share/{package_name}/config", ["config/class_averages.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "radar_fusion_node = radar_ros.radar_fusion_node:main",
        ],
    },
)
