from setuptools import setup, find_packages

package_name = 'cylinder3d_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='todo@todo.com',
    description='ROS 2 wrapper for Cylinder3D',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cylinder3d_node = cylinder3d_ros.cylinder3d_ros:main',
        ],
    },
)
