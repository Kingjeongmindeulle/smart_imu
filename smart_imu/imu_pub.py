import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import numpy as np
from transforms3d.quaternions import quat2mat 
# from tf_transformations import quaternion_matrix -> ros1의 tf 패키지에 포함?
# pip install transforms3d

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
        
        # ✅ 속도 발행 (비중력 가속도 적분)
        self.velocity_pub = self.create_publisher(Vector3, '/imu/velocity', 10)

        # ✅ 상대 위치 발행 (속도 적분)
        self.position_pub = self.create_publisher(Vector3, '/imu/position', 10)

        # ✅ 초기값 설정
        self.prev_time = None
        self.velocity = np.array([0.0, 0.0, 0.0])  # 속도 초기화
        self.position = np.array([0.0, 0.0, 0.0])  # 위치 초기화

        self.get_logger().info("IMU Gravity Processor Node Started!")

    def imu_callback(self, msg):
        """ IMU 데이터를 받아 비중력 계산 + 속도, 위치 """

        current_time = self.get_clock().now().nanoseconds / 1e9  # 초 단위 변환

        # 첫 데이터 수신 시 시간 초기화
        if self.prev_time is None:
            self.prev_time = current_time
            return
        
        # ✅ dt (시간 변화량) 계산
        dt = current_time - self.prev_time
        self.prev_time = current_time

        # ✅ dt 값 보정 (비정상적으로 크거나 0 이하일 경우)
        if dt <= 0 or dt > 0.1:
            self.get_logger().warn(f"⚠️ 비정상적인 dt 감지: {dt:.3f}s → 기본값 0.02s로 보정")
            dt = 0.02  # 50Hz 기준 (현재 100Hz)

        # ✅ 측정된 가속도 데이터 가져오기
        measured_accel = np.array([msg.linear_acceleration.x,
                                   msg.linear_acceleration.y,
                                   msg.linear_acceleration.z])

        # ✅ 쿼터니언을 회전 행렬로 변환 (수정함)
        quaternion = np.array([msg.orientation.w,  
                               msg.orientation.x,
                               msg.orientation.y,
                               msg.orientation.z]) # transforms3d는 (w, x, y, z) 순서 사용
        rotation_matrix = quat2mat(quaternion) # 3x3 회전 행렬 생성


        # ✅ 중력 벡터 변환 (센서 기준으로 변환)
        gravity_earth_frame = np.array([0, 0, 9.81])  # ENU 기준 (필요하면 -9.81로 변경)
        gravity_in_sensor_frame = rotation_matrix @ gravity_earth_frame # 변환 행렬 적용하여 중력 벡터를 센서 좌표계로 변환

        # ✅ 비중력 가속도 계산
        non_gravity_accel = measured_accel - gravity_in_sensor_frame

        # ✅ 속도 계산 (비중력 가속도를 적분)
        self.velocity += non_gravity_accel * dt

        # ✅ 위치 계산 (속도를 적분)
        self.position += self.velocity * dt

        # ✅ 중력 가속도 유지 발행
        gravity_msg = Vector3()
        gravity_msg.x = float(gravity_in_sensor_frame[0])
        gravity_msg.y = float(gravity_in_sensor_frame[1])
        gravity_msg.z = float(gravity_in_sensor_frame[2])
        self.gravity_pub.publish(gravity_msg)

        # ✅ 비중력 가속도 발행
        non_grav_msg = Vector3()
        non_grav_msg.x = float(non_gravity_accel[0])
        non_grav_msg.y = float(non_gravity_accel[1])
        non_grav_msg.z = float(non_gravity_accel[2])
        self.non_gravity_pub.publish(non_grav_msg)

        # ✅ 속도 발행
        velocity_msg = Vector3()
        velocity_msg.x = float(self.velocity[0])
        velocity_msg.y = float(self.velocity[1])
        velocity_msg.z = float(self.velocity[2])
        self.velocity_pub.publish(velocity_msg)

        # ✅ 위치 발행
        position_msg = Vector3()
        position_msg.x = float(self.position[0])
        position_msg.y = float(self.position[1])
        position_msg.z = float(self.position[2])
        self.position_pub.publish(position_msg)

        # ✅ 8. 디버깅 로그 추가 (센서 방향 확인)
        self.get_logger().info(f"""
🌍 Original Acceleration (with gravity): x={measured_accel[0]:.3f}, y={measured_accel[1]:.3f}, z={measured_accel[2]:.3f}
🌍 Rotation Matrix:
{rotation_matrix}
🌍 Transformed Gravity (sensor frame): x={gravity_msg.x:.3f}, y={gravity_msg.y:.3f}, z={gravity_msg.z:.3f}
🌍 Non-Gravity Acceleration: x={non_grav_msg.x:.3f}, y={non_grav_msg.y:.3f}, z={non_grav_msg.z:.3f}
🌍 Velocity: x={velocity_msg.x:.3f}, y={velocity_msg.y:.3f}, z={velocity_msg.z:.3f}
🌍 Position: x={position_msg.x:.3f}, y={position_msg.y:.3f}, z={position_msg.z:.3f}
🕒 Time Step: {dt:.3f}s
        """)

def main():
    rclpy.init()
    node = IMUGravityProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()