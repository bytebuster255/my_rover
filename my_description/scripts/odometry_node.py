#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')

        # --- Rover Parametreleri ---
        self.declare_parameter('wheel_radius', 0.05)      # Tekerlek yarıçapı (m)
        self.declare_parameter('wheel_separation', 0.3)   # Tekerlekler arası mesafe (m)
        self.declare_parameter('ticks_per_rev', 1050.0)    # 1 tam turda okunan tick
        
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.ticks_per_rev = self.get_parameter('ticks_per_rev').value

        # --- Durum (State) Değişkenleri ---
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        # [Sol Ön(1), Sağ Ön(2), Sol Arka(3), Sağ Arka(4)]
        self.ticks = [0, 0, 0, 0] 
        self.prev_ticks = [0, 0, 0, 0]
        
        self.last_time = self.get_clock().now()

        # --- Subscriber'lar (Arduino topic isimleri ile eşleşiyor) ---
        self.create_subscription(Int32, 'encoder_ticks_1', self.tick1_cb, 10)
        self.create_subscription(Int32, 'encoder_ticks_2', self.tick2_cb, 10)
        self.create_subscription(Int32, 'encoder_ticks_3', self.tick3_cb, 10)
        self.create_subscription(Int32, 'encoder_ticks_4', self.tick4_cb, 10)

        # --- Publisher ve TF Broadcaster ---
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # --- Timer (10 Hz - Arduino'nun TIMER_TIMEOUT_MS 100 değeri ile senkronize) ---
        self.timer = self.create_timer(0.1, self.update_odometry)

    # Callback Fonksiyonları
    def tick1_cb(self, msg): self.ticks[0] = msg.data
    def tick2_cb(self, msg): self.ticks[1] = msg.data
    def tick3_cb(self, msg): self.ticks[2] = msg.data
    def tick4_cb(self, msg): self.ticks[3] = msg.data

    def update_odometry(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        if dt == 0:
            return

        # Bir önceki döngüden bu yana değişen tick miktarını bul
        delta_ticks = [self.ticks[i] - self.prev_ticks[i] for i in range(4)]
        self.prev_ticks = list(self.ticks)

        # Sol(1,3) ve Sağ(2,4) taraf için ortalama tick değişimini hesapla (Skid-Steer Mantığı)
        delta_tick_left = (delta_ticks[0] + delta_ticks[2]) / 2.0
        delta_tick_right = (delta_ticks[1] + delta_ticks[3]) / 2.0

        # Tekerleklerin aldığı mesafeler (m)
        m_per_tick = (2.0 * math.pi * self.wheel_radius) / self.ticks_per_rev
        
        d_left = delta_tick_left * m_per_tick
        d_right = delta_tick_right * m_per_tick

        # Robotun merkezinin kat ettiği mesafe ve kendi etrafındaki dönüş açısı (Radyan)
        d_center = (d_left + d_right) / 2.0
        d_theta = (d_right - d_left) / self.wheel_separation

        # Hız (Velocity) hesaplamaları
        v_x = d_center / dt
        v_th = d_theta / dt

        # Global koordinatlardaki yeni pozisyonu güncelle
        self.x += d_center * math.cos(self.th + (d_theta / 2.0))
        self.y += d_center * math.sin(self.th + (d_theta / 2.0))
        self.th += d_theta

        # Yaw açısından Quaternion oluştur
        q = self.quaternion_from_yaw(self.th)

        # ================= TF YAYINI (odom -> base_link) =================
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

        # ================= ODOMETRİ YAYINI (nav_msgs/Odometry) =================
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        # Skid-steer'de y ekseninde kayma (slip) olabilir ama kinematik modelde v_y 0 varsayılır
        odom.twist.twist.linear.x = v_x
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = v_th

        self.odom_pub.publish(odom)
        self.last_time = current_time

    def quaternion_from_yaw(self, yaw):
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return [0.0, 0.0, qz, qw]

def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()