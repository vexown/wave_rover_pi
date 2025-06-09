from setuptools import find_packages, setup

package_name = 'gps_tools'

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
    description='Processing raw uart data, extracting GNSS info from it and republishing it',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'gps_parser = gps_tools.gps_parser_node:main',
        ],
    },
)
