cmake_minimum_required(VERSION 3.8)
project(myimu)

# 빌드 도구
find_package(ament_cmake REQUIRED)

# C++ 노드 의존성
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# 메시지 생성 도구
find_package(rosidl_default_generators REQUIRED)

# (1) 메시지 파일 빌드
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ImuSensor.msg"
  "msg/Calculation.msg"
  "msg/YawPitchRoll.msg"
  "msg/PosVel.msg"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)

# (2) (선택) C++ 노드 예시가 있을 경우에만 추가
# add_executable(imu_publisher src/imu_publisher.cpp)
# ament_target_dependencies(imu_publisher
#   rclcpp std_msgs geometry_msgs builtin_interfaces
# )
# install(TARGETS imu_publisher
#   DESTINATION lib/${PROJECT_NAME}
# )

ament_package()
