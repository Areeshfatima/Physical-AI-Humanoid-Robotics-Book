from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'walking_locomotion'

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
    maintainer='openhack',
    maintainer_email='openhack@todo.todo',
    description='Walking locomotion control for humanoid robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'walking_controller = walking_locomotion.walking_controller:main',
            'ik_solver = walking_locomotion.ik_solver:main',
            'gait_generator = walking_locomotion.gait_generator:main',
        ],
    },
)