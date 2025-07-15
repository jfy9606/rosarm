cmake_minimum_required(VERSION 3.5)
project(serial)

# 设置 C++14 标准
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

# 查找必须的包
find_package(ament_cmake REQUIRED)

# 查找系统依赖
find_package(Threads REQUIRED)

# 设置包含目录
include_directories(include)

# 收集所有源文件
file(GLOB_RECURSE SOURCES src/*.cc)

# 创建库
add_library(${PROJECT_NAME} SHARED ${SOURCES})
target_link_libraries(${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT})

# 设置安装规则
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

# 导出配置
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(Threads)

ament_package()
