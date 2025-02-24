import rclpy
from rclpy.node import Node
from smart_imu.msg import MyImu  # 패키지명과 메시지명을 실제 환경에 맞게 수정하세요

class MyImuTestPublisher(Node):
    def __init__(self):
        super().__init__('my_imu_test_publisher')
        self.publisher_ = self.create_publisher(MyImu, 'my_imu_topic', 10)
        self.timer = self.create_timer(1.0, self.publish_message)  # 1초마다 발행
        self.get_logger().info("MyImu test publisher started")

    def publish_message(self):
        msg = MyImu()
        # Header 설정: 현재 시간과 프레임 ID를 할당
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        
        # 예제 데이터 할당
        msg.orientation_x = 0.0
        msg.orientation_y = 0.0
        msg.orientation_z = 0.0
        msg.orientation_w = 1.0
        
        msg.angular_velocity_x = 0.0
        msg.angular_velocity_y = 0.0
        msg.angular_velocity_z = 0.0
        
        msg.linear_acceleration_x = 0.0
        msg.linear_acceleration_y = 0.0
        msg.linear_acceleration_z = 9.81  # 중력 가속도 예시

        self.publisher_.publish(msg)
        self.get_logger().info("Published MyImu message")

def main(args=None):
    rclpy.init(args=args)
    node = MyImuTestPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
