cmake_minimum_required(VERSION 3.16)
# 指定需要的 CMake 最低版本
project(MyAGVCA LANGUAGES CXX)
# 声明项目名与使用的语言（只启用 C++）

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# 设定 C++ 标准为 C++17，并强制要求编译器支持


find_package(Eigen3 REQUIRED) # 查找 Eigen3 线性代数库
find_package(nlohmann_json REQUIRED) # 查找 nlohmann_json JSON 解析库


find_package(PahoMqttCpp QUIET) # 查找 Paho MQTT C++ 客户端库：
if(NOT PahoMqttCpp_FOUND) # 若未找到，则把项目的 cmake/ 路径加入模块搜索路径，再进行二次查找
  list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
  find_package(PahoMqttCpp REQUIRED)
endif()

include_directories(${CMAKE_SOURCE_DIR}/include) 
# 将项目头文件目录加入编译器的 include 搜索路径

add_library(agv_core
  src/PID.cpp
  src/Kalman.cpp
  src/MqttPahoClient.cpp
  src/Executor.cpp
  src/Log.cpp
)
# 把核心源文件编译为一个库目标 agv_core

target_link_libraries(agv_core   # 为 agv_core 链接第三方依赖：
  Eigen3::Eigen                  # Eigen 头文件库
  nlohmann_json::nlohmann_json   # JSON 库
  PahoMqttCpp::paho-mqttpp3      # Paho MQTT C++ 封装
  PahoMqttCpp::paho-mqtt3as      # Paho MQTT C 异步库
)

add_executable(agv_node apps/main.cpp) # 生成可执行文件 agv_node
target_link_libraries(agv_node agv_core) # 将核心库 agv_core 链接到可执行程序 agv_node 上

