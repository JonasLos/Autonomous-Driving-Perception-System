from setuptools import setup

package_name = 'sam2_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    py_modules=['samv2_image_segmenation','samv2_mask_transform'],
    package_dir={'': 'src'},
    install_requires=['setuptools'],
    zip_safe=True,
)
