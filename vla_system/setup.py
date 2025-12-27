from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'vla_system'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.[py|xml|yaml]')),
        # Include all config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='openhack',
    maintainer_email='openhack@todo.todo',
    description='Vision-Language-Action system for humanoid robot control',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'whisper_node = vla_system.whisper_node.speech_recognition_node:main',
            'llm_planner = vla_system.llm_planner.llm_planner_node:main',
            'navigation_node = vla_system.navigation_node.navigation_node:main',
            'perception_node = vla_system.perception_node.perception_node:main',
            'vla_manager = vla_system.vla_manager.vla_manager_node:main',
            'action_executor = vla_system.action_executor.action_executor_node:main',
        ],
    },
)