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

    # EKF Config dosyasının yolu
    ekf_config_path = os.path.join(pkg_share, 'config', 'ekf.yaml')

    # 1. Robot State Publisher (Gövde ve tekerlek bağlantılarını yayınlar)
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # 2. RViz2 (Görselleştirme - Şu an kapalı, açmak istersen yorumları kaldır)
    # rviz_config_dir = os.path.join(pkg_share, 'rviz', 'urdf.rviz')
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen',
    #     arguments=['-d', rviz_config_dir] 
    # )

    # 3. micro-ROS Agent (F446 köprüsü)
    micro_ros_agent_node = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        output='screen',
        arguments=['serial', '--dev', '/dev/ttyACM0'] # Port numarasına dikkat et
    )
    
    # 4. Joint State Publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    # 5. Odometri Düğümü (STM32'den gelen verilerle odom hesaplar)
    odom_node = Node(
        package='my_description', 
        executable='odometry_node.py', # CMake tarafında uzantısıyla kurulduğu için .py eklendi
        name='odometry_publisher',
        output='screen'
    )

    # 6. IMU Madgwick Filtresi (Ham veriyi yönelime dönüştürür)
    madgwick_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        output='screen',
        parameters=[
            {'use_mag': False} # Manyetometre kapalı
        ],
        remappings=[
            ('/imu/data_raw', '/imu/data_raw') # Dinleyeceği topic
        ]
    )

    # 7. Robot Localization (EKF - Sensör Füzyonu)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path]
    )

    # Tüm düğümler döndürülüyor
    return LaunchDescription([
        node_robot_state_publisher,
        joint_state_publisher_node,
        micro_ros_agent_node,
        odom_node,
        madgwick_filter_node,
        ekf_node
        # rviz_node,  # İhtiyaç duyduğunda aktif edebilirsin
    ])