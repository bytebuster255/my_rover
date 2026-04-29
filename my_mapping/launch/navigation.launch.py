import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share     = get_package_share_directory('my_mapping')
    nav2_bringup  = get_package_share_directory('nav2_bringup')

    map_file_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(os.path.expanduser('~'), 'my_map.yaml'),
        description='Harita dosyası'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Sim zamanı kullan'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_share, 'config', 'nav2_params.yaml'),
        description='Nav2 parametre dosyası'
    )

    map_file     = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file  = LaunchConfiguration('params_file')

    # nav2_bringup'ın kendi localization launch'ını kullan
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'map':           map_file,
            'use_sim_time':  use_sim_time,
            'params_file':   params_file,
        }.items(),
    )

    # nav2_bringup'ın kendi navigation launch'ını kullan
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file':  params_file,
        }.items(),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        map_file_arg,
        use_sim_time_arg,
        params_file_arg,
        localization,
        navigation,
        rviz_node,
    ])