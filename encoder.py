import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32  # Gerekirse Int64 yapabilirsiniz

class XboxTeleopNode(Node):
    def __init__(self):
        super().__init__('xbox_teleop_node')
        
        # Hız parametrelerini tanımlayalım
        self.declare_parameter('max_linear_speed', 1.0)   
        self.declare_parameter('max_angular_speed', 1.0)  
        self.declare_parameter('boost_multiplier', 2.0)   
        self.declare_parameter('calibration_speed', 0.2)  
        
        # Publisher ve Subscriber'lar
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        
        # Encoder Subscriber'ları
        self.sub_enc1 = self.create_subscription(Int32, 'encoder_ticks_1', self.enc1_callback, 10)
        self.sub_enc2 = self.create_subscription(Int32, 'encoder_ticks_2', self.enc2_callback, 10)
        self.sub_enc3 = self.create_subscription(Int32, 'encoder_ticks_3', self.enc3_callback, 10)
        self.sub_enc4 = self.create_subscription(Int32, 'encoder_ticks_4', self.enc4_callback, 10)
        
        # Arm sistemi
        self.is_armed = False
        self.last_x_button_state = 0
        
        # Kalibrasyon Değişkenleri
        self.is_calibrating = False
        self.last_y_button_state = 0
        self.current_ticks = [0, 0, 0, 0]
        self.initial_ticks = [0, 0, 0, 0]
        
        # *** YENİ: Test geçmişini tutacağımız liste ***
        self.test_history = [] 
        
        # Timer (10Hz)
        self.timer = self.create_timer(0.1, self.calibration_timer_callback)

        self.get_logger().info('Xbox Teleop Node başlatıldı.')
        self.get_logger().info('- Aracı sürmek için "X" tuşuna basarak ARM edin.')
        self.get_logger().info('- ENCODER TESTİ için "Y" tuşuna basıp başlatın/durdurun.')

    # --- Encoder Callback Fonksiyonları ---
    def enc1_callback(self, msg): self.current_ticks[0] = msg.data
    def enc2_callback(self, msg): self.current_ticks[1] = msg.data
    def enc3_callback(self, msg): self.current_ticks[2] = msg.data
    def enc4_callback(self, msg): self.current_ticks[3] = msg.data

    def joy_callback(self, msg):
        x_button = msg.buttons[2]
        y_button = msg.buttons[3] 
        rb_button = msg.buttons[5]
        
        # --- Y TUŞU: Encoder Testi ---
        if y_button == 1 and self.last_y_button_state == 0:
            self.is_calibrating = not self.is_calibrating
            
            if self.is_calibrating:
                # Test Başlıyor
                self.initial_ticks = list(self.current_ticks)
                test_no = len(self.test_history) + 1
                self.get_logger().info(f'\n>>> TEST #{test_no} BAŞLADI <<<')
                self.get_logger().info(f'Başlangıç Değerleri: {self.initial_ticks}')
            else:
                # Test Bitiyor
                self.stop_robot()
                diffs = [abs(cur - ini) for cur, ini in zip(self.current_ticks, self.initial_ticks)] # abs() ile negatif dönüşleri de pozitif fark olarak alırız
                
                # Sonucu geçmişe ekle
                self.test_history.append(diffs)
                test_no = len(self.test_history)
                
                # Ortalamaları hesapla
                avg_diffs = [0.0, 0.0, 0.0, 0.0]
                for result in self.test_history:
                    for i in range(4):
                        avg_diffs[i] += result[i]
                
                avg_diffs = [round(total / test_no, 2) for total in avg_diffs]
                
                self.get_logger().info(f'\n>>> TEST #{test_no} BİTTİ <<<')
                self.get_logger().info(f'Bu Testteki Fark: {diffs}')
                self.get_logger().info(f'--- GENEL İSTATİSTİKLER ({test_no} Test) ---')
                self.get_logger().info(f'Tüm Testlerin Geçmişi: {self.test_history}')
                self.get_logger().info(f'>>> BAZ ALINACAK ORTALAMA DEĞER: {avg_diffs} <<<')
                
        self.last_y_button_state = y_button

        if self.is_calibrating:
            return

        # --- X TUŞU: Arm/Disarm ---
        if x_button == 1 and self.last_x_button_state == 0:
            self.is_armed = not self.is_armed
            status = "ARMED" if self.is_armed else "DISARMED"
            self.get_logger().info(f'Araç Durumu Değişti: {status}')
            if not self.is_armed:
                self.stop_robot()
        self.last_x_button_state = x_button

        # --- NORMAL SÜRÜŞ ---
        if self.is_armed:
            twist = Twist()
            base_linear = self.get_parameter('max_linear_speed').value
            base_angular = self.get_parameter('max_angular_speed').value
            current_multiplier = self.get_parameter('boost_multiplier').value if rb_button == 1 else 1.0
            
            twist.linear.x = msg.axes[1] * base_linear * current_multiplier
            twist.angular.z = msg.axes[3] * base_angular * current_multiplier
            self.publisher_.publish(twist)

    def calibration_timer_callback(self):
        if self.is_calibrating:
            twist = Twist()
            twist.linear.x = self.get_parameter('calibration_speed').value
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
            
            # Anlık farkları mutlak değer (abs) olarak gösteriyoruz ki eksi/artı kafa karıştırmasın
            diffs = [abs(cur - ini) for cur, ini in zip(self.current_ticks, self.initial_ticks)]
            self.get_logger().info(f'[TEST #{len(self.test_history)+1} AKTİF] Anlık Fark: {diffs}')

    def stop_robot(self):
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
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()