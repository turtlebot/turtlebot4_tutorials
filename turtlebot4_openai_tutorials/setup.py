import os
from glob import glob
from setuptools import setup

package_name = 'turtlebot4_openai_tutorials'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # Marker file
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Package.xml
        ('share/' + package_name, ['package.xml']),
        # Launch files
<<<<<<< HEAD
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*launch.py'))),
        # Prompt files
        (os.path.join('share', package_name, 'prompts'),
         glob(os.path.join('prompts', '*.txt'))),
=======
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),
        # Prompt files
        (os.path.join('share', package_name, 'prompts'), glob(os.path.join('prompts', '*.txt'))),
>>>>>>> Working example
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rgariepy',
    maintainer_email='rgariepy@clearpath.ai',
    description='TurtleBot 4 OpenAI Integration Tutorials',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nav_to_pose = turtlebot4_openai_tutorials.nav_to_pose:main',
            'natural_language_nav = \
                turtlebot4_openai_tutorials.natural_language_nav:main',
        ],
    },
)
