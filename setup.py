import os
from glob import glob
from setuptools import setup

package_name = 'rr1_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='chudyja1@fit.cvut.cz',
    description='ROS2 package for desktop robotic arm RR1 containing the control implementation.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "test_trajectory = rr1_control.controller_test:main"
        ],
    },
)
