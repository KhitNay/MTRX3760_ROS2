cmake_minimum_required(VERSION 3.8)
project(mtrx3760)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(simplePublisher src/simple_publisher.cpp)
ament_target_dependencies(simplePublisher rclcpp geometry_msgs)

add_executable(simpleSubscriber src/simple_subscriber.cpp)
ament_target_dependencies(simpleSubscriber rclcpp geometry_msgs)

install(TARGETS
  simplePublisher
  simpleSubscriber
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
