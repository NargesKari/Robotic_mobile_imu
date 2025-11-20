from setuptools import setup
import os
from glob import glob

package_name = 'sensor'

executable_files = [
    'imu_pub.py',
    'imu_filter.py',
    'imu_odometry.py',
]

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[py]*'))),

    ],
    install_requires=['setuptools', 'numpy'], # Include numpy dependency
    zip_safe=True,
    maintainer='dynamic',
    maintainer_email='dynamic@todo.todo',
    description='IMU Sensor Data Processing and Odometry Estimation',
    license='TODO: License declaration',
    tests_require=['pytest'],
    
    entry_points={
        'console_scripts': [
            'imu_pub = sensor.imu_pub:main', 
            'imu_filter = sensor.imu_filter:main', 
            'imu_odometry = sensor.imu_odometry:main',
        ],
    },
)