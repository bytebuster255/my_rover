import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share = get_package_share_directory('my_mapping')

    # --- Launch argümanları ---
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',          # Gazebo için true, gerçek robot için false
        description='Gazebo simülasyon saatini kullan'
    )

    slam_params_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(pkg_share, 'config', 'mapper_params.yaml'),
        description='slam_toolbox parametre dosyası'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')

    # --- slam_toolbox: async harita oluşturucu ---
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ],
    )

    # --- RViz2: haritalandırmayı görselleştir ---
    rviz_config = os.path.join(pkg_share, 'config', 'mapping.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        use_sim_time_arg,
        slam_params_arg,
        slam_toolbox_node,
        rviz_node,
    ])