#Storage for Yaw, Pitch, and Roll values computed from each sensor
#made by kimjeongmindeulle

std_msgs/Header header

# 가속도
YawPitchRoll accel

# 자이로스코프 (각속도)
YawPitchRoll gyro

# 자기장 센서 값
YawPitchRoll magnetic

# 최종 보정된 값
YawPitchRoll final
