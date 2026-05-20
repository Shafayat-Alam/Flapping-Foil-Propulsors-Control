from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'compliant_propulsors_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Shafayat Alam',
    maintainer_email='your_email@example.com',
    description='ROS2 control stack for bio-inspired flapping foil propulsors',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'crab = compliant_propulsors_control.crab:main',
            'controller = compliant_propulsors_control.controller:main',
            'Dynamixel_XW430_T200_interface = compliant_propulsors_control.Dynamixel_XW430_T200_interface:main',
            'icm20948_interface = compliant_propulsors_control.icm20948_interface:main',
            'stellarhd_interface = compliant_propulsors_control.stellarhd_interface:main',
            'gazebo_dynamixel_interface = compliant_propulsors_control.gazebo_dynamixel_interface:main',
            'gazebo_icm20948_interface = compliant_propulsors_control.gazebo_icm20948_interface:main',
            'gazebo_stellarhd_interface = compliant_propulsors_control.gazebo_stellarhd_interface:main',
        ],
    },
)