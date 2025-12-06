from setuptools import setup
import os
from glob import glob

package_name = 'robot'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Ben',
    author_email='lkxben@gmail.com',
    description='Object tracking robot ROS 2 package',
    entry_points={
        'console_scripts': [
            'stream_to_image_node = robot.nodes.stream_to_image_node:main',
            'annotated_publisher = robot.nodes.annotated_publisher:main',
            'tracking_node = robot.nodes.tracking_node:main',
            'prompt_cv_node = robot.nodes.prompt_cv_node:main',
            'mot_node = robot.nodes.mot_node:main',
            'sot_node = robot.nodes.sot_node:main',
            'state_manager = robot.nodes.state_manager:main',
            'logging_node = robot.nodes.logging_node:main'
        ],
    }, data_files=[
        ('share/ament_index/resource_index/packages',
            ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('robot/launch/*.py')),
    ],
)