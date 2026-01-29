from setuptools import setup

package_name = 'clrernet_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    py_modules=['clrernet_lane_detection','clrernet_lane_transform','inference'],
    package_dir={'': 'src'},
    install_requires=['setuptools'],
    zip_safe=True,
)
