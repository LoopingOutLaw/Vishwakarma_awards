from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'arm_vla_pkg'

setup(
    name=package_name,
    version='0.0.0',
    # This line is NEW. It tells setuptools to find your 'nodes' folder.
    packages=[package_name, 'arm_vla_pkg.nodes'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='harshit',
    maintainer_email='saxena150723@gmail.com',
    description='VLA logic for 6-DOF arm',
    license='TODO: License declaration',
    tests_require=['pytest'],
    # This section is UPDATED to include ALL your nodes.
    entry_points={
        # Change your entry_points to look like this:
        'console_scripts': [
            'action_node = arm_vla_pkg.nodes.action_node:main',
            'vision_node = arm_vla_pkg.nodes.vision_node:main',
            'speech_node = arm_vla_pkg.nodes.speech_node:main',
            'brain_node = arm_vla_pkg.nodes.brain_node:main',
        ],
    },
)