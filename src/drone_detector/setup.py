from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'drone_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yy',
    maintainer_email='2460220279@qq.com',
    description='YOLOv8 无人机识别节点',
    license='TODO',
    tests_require=['pytest'],
    scripts=[os.path.join('scripts', 'drone_detector_node')],
)
