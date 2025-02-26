# This is a message to hold data from an IMU (Inertial Measurement Unit), made by kimjeongmindeulle
#
# Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec
# Magnetic field should be in uT
#
# If the covariance of the measurement is known, it should be filled in (if all you know is the
# variance of each measurement, e.g. from the datasheet, just put those along the diagonal)
# A covariance matrix of all zeros will be interpreted as "covariance unknown"
#
# If you have no estimate for one of the data elements (e.g. your IMU doesn't produce an
# orientation estimate), please set element 0 of the associated covariance matrix to -1

# ImuSensor.msg

# IMU 메시지 예시

# 표준 헤더 (stamp, frame_id 등)
std_msgs/Header header

# Orientation (쿼터니언)
geometry_msgs/Quaternion orientation

# 선가속도
geometry_msgs/Vector3 linear_acceleration

# 각속도
geometry_msgs/Vector3 angular_acceleration

# 자기장
geometry_msgs/Vector3 magnetic_field
