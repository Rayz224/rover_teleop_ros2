import os

from setuptools import setup
from glob import glob

package_name = 'rover_teleop'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Raphael DL',
    maintainer_email='raphael.dl224@gmail.com',
    description='Teleop for the wave rover',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rover_teleop = rover_teleop.rover_teleop:main'
        ],
    },
)
