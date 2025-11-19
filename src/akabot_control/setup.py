from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'akabot_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.py')),
    ],
    install_requires=[
        'setuptools',
        'opencv-python',
        'numpy',
        'scipy',
    ],
    zip_safe=True,
    maintainer='aditya',
    maintainer_email='aditya.arora.emails@gmail.com',
    description='Akabot control package with vision',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'akabot_controller = akabot_control.akabot_controller:main',
            'interactive_control = akabot_control.interactive_control:main',
            'vision_pick_place = akabot_control.vision_pick_place:main',
            'scanning_vision_pick_place = akabot_control.scanning_vision_pick_place:main',
            'test_ee_camera = akabot_control.test_ee_camera:main',
        ],
    },
)