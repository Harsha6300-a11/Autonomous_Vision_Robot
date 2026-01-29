cmake_minimum_required(VERSION 3.8)
project(vision_person_follower)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TrackedPerson.msg"
)

# Install Python node executables
install(PROGRAMS
  vision_person_follower/perception_node.py
  vision_person_follower/controller_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
