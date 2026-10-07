from glob import glob
from setuptools import find_packages, setup

package_name = 'multi_tracking'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test', 'test.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        ('share/' + package_name + '/resource', [
            'resource/tracking.yaml',
            'resource/adaboost_trained_data_mess_430.txt',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jian960810-hub',
    maintainer_email='jian960810@gmail.com',
    description='TurtleBot3 LiDAR leg detection, multi-person tracking and scan recording.',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'real_time_6_5 = multi_tracking.real_time_6_5:main',
            'getdata = multi_tracking.getdata:main',
        ],
    },
)
