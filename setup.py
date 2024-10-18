from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'yolo'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*_launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yohei yanagi',
    maintainer_email='yanagi0214yohei@gmail.com',
    description='a package for sensor',
    license='BSD',
    #tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_image_node = yolo.yolo_image_node:main',
            'yolo_trafficlight_node = yolo.yolo_trafficlight_node:main',
            'control_node = yolo.control_node:main',
            
        ],
    },
)
