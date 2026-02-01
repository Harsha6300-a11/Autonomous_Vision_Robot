from setuptools import setup

package_name = 'vision_person_follower'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            'share/' + package_name + '/launch',
            ['launch/vision_person_follower.launch.py']
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Harsha',
    maintainer_email='harsha@example.com',
    description='ROS 2 vision-based person follower using YOLOv8, DeepSORT, and LiDAR',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_follower = vision_person_follower.controller_node:main',
        ],
    },
)
