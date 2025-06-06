from setuptools import find_packages, setup

package_name = 'my_camera_pkg'

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
    description='Package for receiving Camera Module 3 feed from Raspberry Pi',
    license='GPL 3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'image_subscriber = my_camera_pkg.image_subscriber:main',
        ],
    },
)
