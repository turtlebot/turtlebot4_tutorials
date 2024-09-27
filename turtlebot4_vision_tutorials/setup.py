from glob import glob
import os

from setuptools import setup

package_name = 'turtlebot4_vision_tutorials'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'models'), glob('models/*.blob')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'templates'), glob('templates/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hilary-luo',
    maintainer_email='hluo@clearpathrobotics.com',
    description='TurtleBot 4 Vision Tutorials',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pose_detection = turtlebot4_vision_tutorials.pose_detection:main',
            'pose_display = turtlebot4_vision_tutorials.pose_display:main',
        ],
    },
)
