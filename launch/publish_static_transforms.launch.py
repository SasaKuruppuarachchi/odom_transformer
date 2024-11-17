#    Odom transformer - ROS 2 Node for performing odometry transformations.
#    Copyright (C) 2023  Karelics Oy
#
#    This program is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with this program. If not, see <https://www.gnu.org/licenses/>.

"""Example launch file for tansform_pub"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Launch tansform_pub. -> translation + orientation + [parent, child]"""
    
    package_share_path = get_package_share_directory("odom_transformer")
    transformer_config_path = os.path.join(package_share_path, "params", "odom_transformer_params.yaml")
    
    px4_imu = Node(
        package="odom_transformer",
        executable="px4_imu_node.py",
        name="px4_imu",
        output={"both": {"screen", "log", "own_log"}},
        parameters=[],
    )
    odom_transformer = Node(
        package="odom_transformer",
        executable="transformer_node.py",
        name="odom_transformer",
        output={"both": {"screen", "log", "own_log"}},
        parameters=[transformer_config_path],
    )
    odom_to_path = Node(
        package="odom_transformer",
        executable="odom_to_path_node.py",
        name="odom_to_path",
        output={"both": {"screen", "log", "own_log"}},
        parameters=[],
    )
    # Map
    transform_drone0map_to_camerainit = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_base_pub',
        output='screen',
        arguments = [ '0.0795', '0', '0.0323', '0', '0.3826834', '0', '0.9238795', 'drone0/map', 'camera_init'] 
    )
    # Lidar IMU
    transform_body_to_livox_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=['0.011', '0.02329', '-0.04412', '0', '0', '0', '1', 'body', 'livox_frame']
    )
    # Camera front
    transform_px4_to_camera_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=['0.117', '0', '-0.025', '-0.5', '0.5', '-0.5', '0.5', 'odom_base', 'camera']
    )
    transform_body_to_odombase_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments = [ '-0.0795', '0', '-0.0323', '0', '0', '0', '1', 'lidar_base', 'odom_base'] 
        #arguments=['-0.0795', '0', '-0.0323', '0', '0', '0', '1', 'body', 'odom_base']
    )
    
    transform_body_to_lidarbase_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments = [ '0', '0', '0', '0', '-0.3826834', '0', '0.9238795', 'body', 'lidar_base'] 
        #arguments=['-0.0795', '0', '-0.0323', '0', '0', '0', '1', 'body', 'odom_base']
    )
    # px4
    transform_baselink_to_px4_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '1', '0', '0', '0', 'odom_base', 'px4_frame']
    )
    
    
    

    return LaunchDescription([odom_transformer,px4_imu, odom_to_path, transform_drone0map_to_camerainit, \
        transform_px4_to_camera_frame, \
        transform_body_to_livox_frame,\
        transform_body_to_odombase_frame, transform_body_to_lidarbase_frame,\
        transform_baselink_to_px4_frame]\
        )
