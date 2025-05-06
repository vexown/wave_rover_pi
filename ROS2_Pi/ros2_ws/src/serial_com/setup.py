from setuptools import find_packages, setup

package_name = 'serial_com'

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
    maintainer='blankmcu',
    maintainer_email='vexown@gmail.com',
    description='A simple ROS2 node for serial communication',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'serial_node = serial_com.serial_node:main',
        ],
    },
)
