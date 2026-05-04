import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class XboxTeleopNode(Node):
    def __init__(self):
        super().__init__('xbox_teleop_node')
        
        # Hız parametrelerini tanımlayalım
        self.declare_parameter('max_linear_speed', 1.0)   # Normal İleri/Geri maksimum hız (m/s)
        self.declare_parameter('max_angular_speed', 1.0)  # Normal Sağ/Sol maksimum dönüş hızı (rad/s)
        self.declare_parameter('boost_multiplier', 2.0)   # RB tuşuna basıldığında hızın kaç katına çıkacağı
        
        # Publisher (Araç için) ve Subscriber (Gamepad için)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        
        # Arm sistemi için durum değişkenleri
        self.is_armed = False
        self.last_x_button_state = 0
        
        self.get_logger().info('Xbox Teleop Node başlatıldı.')
        self.get_logger().info('- Aracı sürmek için "X" tuşuna basarak ARM edin.')
        self.get_logger().info('- Hızlanmak (Power/Boost) için "RB" tuşuna basılı tutun.')
        self.get_logger().info('- NOT: Sağ/Sol yönü tersine çevrildi.')

    def joy_callback(self, msg):
        # Gamepad Mapping:
        # axes[1]: Sol Joystick Y ekseni (Yukarı/Aşağı) -> İleri/Geri
        # axes[3]: Sağ Joystick X ekseni (Sol/Sağ) -> Sağ/Sol (Ters Çevrildi)
        # buttons[2]: X tuşu -> Arm/Disarm
        # buttons[5]: RB tuşu -> Power/Boost
        
        x_button = msg.buttons[2]
        rb_button = msg.buttons[5]
        
        # X tuşuna "ilk basıldığı" anı yakalamak için
        if x_button == 1 and self.last_x_button_state == 0:
            self.is_armed = not self.is_armed
            status = "ARMED (Sürüşe Hazır)" if self.is_armed else "DISARMED (Kilitli)"
            self.get_logger().info(f'Araç Durumu Değişti: {status}')
            
            # Araç disarm edildiğinde aniden durması için 0 komutu gönderelim
            if not self.is_armed:
                self.stop_robot()
                
        self.last_x_button_state = x_button

        # Eğer araç ARMED durumdaysa joy verilerini cmd_vel'e dönüştür
        if self.is_armed:
            twist = Twist()
            
            # Hız limitlerini al
            base_linear = self.get_parameter('max_linear_speed').value
            base_angular = self.get_parameter('max_angular_speed').value
            
            # RB tuşuna basılıysa hızı katla, değilse normal hızda (1.0 çarpanı) bırak
            current_multiplier = self.get_parameter('boost_multiplier').value if rb_button == 1 else 1.0
            
            max_linear = base_linear * current_multiplier
            max_angular = base_angular * current_multiplier
            
            # Sol Joystick İleri/Geri -> Doğrusal hız (Linear X)
            twist.linear.x = msg.axes[1] * max_linear
            
            # Sağ Joystick Sağ/Sol -> Açısal hız (Angular Z) 
            # NOT: Başına - eklenerek yön TERS ÇEVRİLDİ
            twist.angular.z = - (msg.axes[3] * max_angular)
            
            self.publisher_.publish(twist)

    def stop_robot(self):
        # Aracı durdurmak için boş Twist mesajı gönderiyoruz
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = XboxTeleopNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot() # Düğüm kapanırken aracı güvenli şekilde durdur
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()