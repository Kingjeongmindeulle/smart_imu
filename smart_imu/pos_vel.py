import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import numpy as np
from tf_transformations import quaternion_matrix  # ✅ 올바른 임포트 방식

class IMUGravityProcessor(Node):
    def __init__(self):
        super().__init__('imu_gravity_processor')

        # ✅ 기존 IMU 데이터 구독
        self.imu_sub = self.create_subscription(
            Imu, '/android/imu', self.imu_callback, 10)

        # ✅ 기존 중력 가속도 유지하는 토픽 발행
        self.gravity_pub = self.create_publisher(
            Vector3, '/imu/with_gravity', 10)

        # ✅ 비중력 가속도 (중력 제거 후) 발행
        self.non_gravity_pub = self.create_publisher(
            Vector3, '/imu/non_gravity_acceleration', 10)

        self.get_logger().info("IMU Gravity Processor Node Started!")

    def imu_callback(self, msg):
        """ IMU 데이터를 받아 기존 중력 가속도를 유지하면서 비중력 가속도 계산 """
        # ✅ 1. 측정된 가속도 데이터 가져오기
        measured_accel = np.array([msg.linear_acceleration.x,
                                   msg.linear_acceleration.y,
                                   msg.linear_acceleration.z])

        # ✅ 2. 쿼터니언을 회전 행렬로 변환
        quaternion = np.array([msg.orientation.x,
                               msg.orientation.y,
                               msg.orientation.z,
                               msg.orientation.w])

        # ✅ 3. 회전 행렬 변환
        rotation_matrix = quaternion_matrix(quaternion)[:3, :3]

        # ✅ 4. 중력 벡터 변환 (센서 기준으로 변환)
        # 휴대폰이 뒤집혔을 때 자동으로 중력 방향 조정
        gravity_earth_frame = np.array([0, 0, 9.81])  # 일반적으로 ENU 기준 (필요하면 -9.81로 변경)
        
        # 변환 행렬 적용하여 중력 벡터를 센서 좌표계로 변환
        gravity_in_sensor_frame = rotation_matrix @ gravity_earth_frame

        # ✅ 5. 비중력 가속도 계산 (센서에서 측정된 값 - 중력)
        non_gravity_accel = measured_accel - gravity_in_sensor_frame

        # ✅ 6. 기존 중력 가속도 유지하여 발행
        gravity_msg = Vector3()
        gravity_msg.x = float(gravity_in_sensor_frame[0])
        gravity_msg.y = float(gravity_in_sensor_frame[1])
        gravity_msg.z = float(gravity_in_sensor_frame[2])
        self.gravity_pub.publish(gravity_msg)

        # ✅ 7. 비중력 가속도 발행
        non_grav_msg = Vector3()
        non_grav_msg.x = float(non_gravity_accel[0])
        non_grav_msg.y = float(non_gravity_accel[1])
        non_grav_msg.z = float(non_gravity_accel[2])
        self.non_gravity_pub.publish(non_grav_msg)

        # ✅ 8. 디버깅 로그 추가 (센서 방향 확인)
        self.get_logger().info(f"""
📌 Original Acceleration (with gravity): x={measured_accel[0]:.3f}, y={measured_accel[1]:.3f}, z={measured_accel[2]:.3f}
🌀 Rotation Matrix:
{rotation_matrix}
🌍 Transformed Gravity (sensor frame): x={gravity_msg.x:.3f}, y={gravity_msg.y:.3f}, z={gravity_msg.z:.3f}
🚀 Non-Gravity Acceleration: x={non_grav_msg.x:.3f}, y={non_grav_msg.y:.3f}, z={non_grav_msg.z:.3f}
        """)

def main():
    rclpy.init()
    node = IMUGravityProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
