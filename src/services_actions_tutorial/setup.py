from setuptools import setup
import os
from glob import glob

package_name = 'services_actions_tutorial'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='todo',
    maintainer_email='todo@todo.todo',
    description='Tutorials for ROS 2 services and actions',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'add_two_ints_server = services_actions_tutorial.add_two_ints_server:main',
            'add_two_ints_client = services_actions_tutorial.add_two_ints_client:main',
            'fibonacci_action_server = services_actions_tutorial.fibonacci_action_server:main',
            'fibonacci_action_client = services_actions_tutorial.fibonacci_action_client:main',
        ],
    },
)