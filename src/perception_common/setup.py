from setuptools import setup

package_name = "perception_common"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{package_name}"],
        ),
        (f"share/{package_name}", ["package.xml", "topics.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
)
