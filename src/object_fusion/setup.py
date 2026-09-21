from setuptools import setup

package_name = "object_fusion"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", ["config/topics.yaml", "config/object_fusion.rviz"]),
        (f"share/{package_name}/launch", ["launch/object_fusion.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "ego_frame_publisher = object_fusion.ego_frame_publisher_node:main",
            "camera_lidar_detector = object_fusion.camera_lidar_detector_node:main",
            "radar_detector = object_fusion.radar_detector_node:main",
            "ground_projection = object_fusion.ground_projection_node:main",
            "lidar_cluster_detector = object_fusion.lidar_cluster_detector_node:main",
            "object_aggregator = object_fusion.object_aggregator_node:main",
        ],
    },
)
