from setuptools import find_packages, setup

package_name = 'beetlebot_localization'
import os
from glob import glob

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),     
        (os.path.join('share', package_name, 'map'), glob('map/*')),    
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kronton',
    maintainer_email='kyrlosfekry@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'initial_pose_pub = beetlebot_localization.initial_pose_pub.py:main',
        ],
    },
)
