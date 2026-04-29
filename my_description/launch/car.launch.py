import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    # Paket ismi (Kendi paketinin adıyla değiştir)
    pkg_name = 'my_description'
    pkg_share = get_package_share_directory(pkg_name)
    
    # Xacro dosyasının yolunu bul ve işle
    xacro_file = os.path.join(pkg_share, 'urdf', 'rover.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # 1. Robot State Publisher (Gövde ve tekerlek bağlantılarını yayınlar)
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # 2. RViz2 (Görselleştirme)
    # Rviz ayar dosyan varsa yorum satırını kaldırıp yolunu belirtebilirsin
    # rviz_config_dir = os.path.join(pkg_share, 'rviz', 'urdf.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        # arguments=['-d', rviz_config_dir] 
    )

    # 3. micro-ROS Agent (F446 köprüsü)
    micro_ros_agent_node = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        output='screen',
        arguments=['serial', '--dev', '/dev/ttyACM0'] # Port numarasına dikkat et
    )
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    # 4. Odometri Düğümü (STM32'den gelen verilerle odom hesaplar)
    odom_node = Node(
        package='my_description', 
        executable='odometry_node.py', # CMake tarafında uzantısıyla kurulduğu için .py eklendi
        name='odometry_publisher',
        output='screen'
    )

    # Tüm düğümler döndürülüyor
    return LaunchDescription([
        node_robot_state_publisher,
        joint_state_publisher_node,
      #  rviz_node,
        micro_ros_agent_node,
        odom_node
    ])