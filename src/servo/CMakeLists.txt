cmake_minimum_required(VERSION 3.10...3.25)
project(servo)

# 默认C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 设置 CMP0148 策略以解决 Python 查找警告
if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(serial REQUIRED)

# 创建简单的示例程序
add_executable(serial_example
  src/serial_example.cpp
)
target_include_directories(serial_example PUBLIC
  ${serial_INCLUDE_DIRS}
)
target_link_libraries(serial_example
  ${serial_LIBRARIES}
)

# 创建模拟串口示例程序
add_executable(serial_mock
  src/serial_mock.cpp
)

# 创建模拟舵机控制示例程序
add_executable(servo_mock
  src/servo_mock.cpp
)

# 创建独立舵机控制示例程序
add_executable(standalone_servo_control
  src/standalone_servo_control.cpp
)

# 安装可执行文件
install(
  TARGETS serial_example serial_mock servo_mock standalone_servo_control
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
