from setuptools import find_packages, setup
import os 
from glob import glob


package_name = 'nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch'),glob('launch/*')),
        (os.path.join('share',package_name,'config'),glob('config/*')),
        (os.path.join('share',package_name,'maps'),glob('maps/*')),
        (os.path.join('share',package_name,'rviz2'),glob('rviz2/*')),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='faheem',
    maintainer_email='Faheem1243@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'save_pose = nav.save_pose:main',
            'navigate_through_waypoints = nav.c_navigate_through_waypoints:main',
            'navigate_to_pose = nav.c_navigate_to_pose:main',
        ],
    },
)
