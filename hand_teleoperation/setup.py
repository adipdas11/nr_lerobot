from setuptools import setup
import os
from glob import glob

package_name = 'hand_teleoperation'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'data'), glob('data/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Pau M. Marcos Aguilar',
    maintainer_email='paumaria07@gmail.com',
    description='Teleoperated robotic hand',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'delay_pub = hand_teleoperation.delay_pub:main',
            #'delay_capture = hand_teleoperation.delay_capture:main',
            'mediapipe_node = hand_teleoperation.mediapipe_node:main',
            'filter_node = hand_teleoperation.filter_node:main',
        ],
    },
)
