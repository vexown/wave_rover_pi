from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'visual_odo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='blankmcu',
    maintainer_email='vexown@gmail.com',
    description='Simple Visual Odometry package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'compressed_to_raw = visual_odo.compressed_to_raw:main',
            'simple_vo = visual_odo.simple_vo:main',
        ],
    },
)
