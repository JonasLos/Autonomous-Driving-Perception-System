from setuptools import setup

package_name = 'sphereformer_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    py_modules=['sphereformer_ros','semantic_kitti_ros'],
    package_dir={'': 'src'},
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'sphereformer_ros = sphereformer_ros:main',
        ],
    },
)
