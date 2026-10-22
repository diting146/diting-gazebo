"""Run Point-LIO 3D mapping without Nav2 planning or control servers."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    params = ParameterFile(RewrittenYaml(
        source_file=LaunchConfiguration('params_file'),
        root_key='',
        param_rewrites={'use_sim_time': LaunchConfiguration('use_sim_time')},
        convert_types=True,
    ), allow_substs=True)
    return LaunchDescription([
        DeclareLaunchArgument('params_file'),
        DeclareLaunchArgument('use_sim_time', default_value='True'),
        Node(package='point_lio', executable='pointlio_mapping', name='point_lio',
             parameters=[params, {'prior_pcd.enable': False,
                                 'pcd_save.pcd_save_en': True,
                                 'pcd_save.interval': -1,
                                 'odometry.publish_odometry_without_downsample': False}],
             output='screen'),
        Node(package='loam_interface', executable='loam_interface_node',
             name='loam_interface', parameters=[params], output='screen'),
        Node(package='sensor_scan_generation', executable='sensor_scan_generation_node',
             name='sensor_scan_generation', parameters=[params], output='screen'),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='mapping_map_to_odom',
             arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
             parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
             output='screen'),
    ])
