import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    # Paket ismi
    pkg_name = 'my_description'
    pkg_share = get_package_share_directory(pkg_name)
    
    # ================= YENİ: World Dosyası =================
    # 'worlds' klasörü içindeki 'sadit.world' dosyasının yolunu belirliyoruz
    world_file_path = os.path.join(pkg_share, 'worlds', 'sadit.world')
    # =======================================================

    # Xacro dosyasının yolunu bul ve işle
    xacro_file = os.path.join(pkg_share, 'urdf', 'rover.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # Robot State Publisher düğümü
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # ================= GÜNCELLEME: Gazebo =================
    # Gazebo başlatılırken özel world dosyasını argüman olarak gönderiyoruz
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world_file_path}.items()
    )
    # =======================================================

    # Robotu Gazebo ortamına (spawn) dahil et
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'rover',
                                   # İpucu: Robotun haritadaki başlangıç konumunu (Z ekseninde biraz yukarıda) ayarlıyoruz
                                   '-x', '0.0', '-y', '0.0', '-z', '0.5'],
                        output='screen')

    # Tüm düğümleri ve eylemleri döndür
    return LaunchDescription([
        node_robot_state_publisher,
        gazebo,
        spawn_entity
    ])