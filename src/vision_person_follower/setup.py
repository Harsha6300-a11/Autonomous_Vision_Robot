from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'vision_person_follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),

    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'msg'),
            glob('msg/*.msg')),
    ],

    install_requires=['setuptools'],
    zip_safe=True,

    maintainer='harsha',
    maintainer_email='harshachintagunta@gmail.com',
    description='Vision-based person follower using YOLO and DeepSORT',
    license='Apache-2.0',

    entry_points={
        'console_scripts': [
            'perception_node = vision_person_follower.perception_node:main',
            'controller_node = vision_person_follower.controller_node:main',
        ],
    },
)
