from setuptools import find_packages, setup

package_name = 'arm_vla_pkg'

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
    maintainer='harshit',
    maintainer_email='harshit@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'action_node = arm_vla_pkg.nodes.action_node:main',
            'vision_node = arm_vla_pkg.nodes.vision_node:main',
            'speech_node = arm_vla_pkg.nodes.speech_node:main',
            'brain_node = arm_vla_pkg.nodes.brain_node:main',
        ],
    },
)
