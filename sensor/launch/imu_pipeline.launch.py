from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os

ACCEL_BIAS_X = 0.1505710744857788
ACCEL_BIAS_Y = -0.017330941036343573
ACCEL_BIAS_Z = -2.32152099609375
GYRO_BIAS_X = 0.0020036364789120854
GYRO_BIAS_Y = 0.0020280710776569323
GYRO_BIAS_Z = 0.0011606430751271545
# -------------------------------------------------------------------

def generate_launch_description():
    package_name = 'sensor'
    pkg_share_dir = get_package_share_directory(package_name)
    rviz_config_file = os.path.join(pkg_share_dir, 'config', 'path.rviz')
    
    publisher_node = Node(
        package=package_name,
        executable='imu_publisher', 
        name='imu_pub',
        output='screen',
    )

    filter_node = Node(
        package=package_name,
        executable='imu_filter', 
        name='imu_filter',
        output='screen',
        parameters=[
            {'cutoff_frequency': 5.0},
            {'accel_bias.x': ACCEL_BIAS_X},
            {'accel_bias.y': ACCEL_BIAS_Y},
            {'accel_bias.z': ACCEL_BIAS_Z},
            {'gyro_bias.x': GYRO_BIAS_X},
            {'gyro_bias.y': GYRO_BIAS_Y},
            {'gyro_bias.z': GYRO_BIAS_Z},
        ],
        ros_arguments=['--log-level', 'WARN']
    )

    odometry_node = Node(
        package=package_name,
        executable='imu_odometry', 
        name='imu_odometry',
        output='screen',
        ros_arguments=['--log-level', 'WARN']
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_visualizer',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        # publisher_node,
        filter_node,
        odometry_node,
        rviz_node
    ])