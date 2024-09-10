from setuptools import find_packages, setup

package_name = 'yolo'

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
    maintainer='yohei yanagi',
    maintainer_email='yanagi0214yohei@gmail.com',
    description='a package for sensor',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_node = yolo.yolo_node:main'
        ],
    },
)
