from setuptools import setup
import os
from glob import glob

package_name = 'publisher_subscriber_tutorial'

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
    description='Tutorials for ROS 2 publisher and subscriber nodes',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = publisher_subscriber_tutorial.talker_listener:main',
            'listener = publisher_subscriber_tutorial.subscriber:main',
        ],
    },
)