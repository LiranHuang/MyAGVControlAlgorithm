#pragma once
#include "agv/RobotState.hpp"     // 机器人状态结构定义
#include "agv/IMqttClient.hpp"    // 通讯接口定义 MQTT
#include "agv/PID.hpp"
#include "agv/Kalman.hpp"
#include "agv/Config.hpp"         // 配置参数
#include <string>
#include <mutex>

namespace agv {   // 命名空间 agv，避免命名冲突
class Executor {   // 定义执行器类
private:
  IMqttClient& mqtt_;  // 通讯接口引用
  Config cfg_;  // 系统配置参数
  PID pid_vx_, pid_wz_;  // PID 控制器：线速度和角速度
  Kalman1D kf_x_, kf_y_, kf_yaw_;  // 卡尔曼滤波器，用于估计位姿
  RobotState state_;  // 当前机器人状态
  std::mutex mtx_;   // 互斥锁，用于线程安全
public: 
  Executor(IMqttClient& mqtt, const Config& cfg);  // 构造函数：初始化控制器，注入通信接口与配置
  void init();  // 初始化函数：配置订阅、状态变量、PID 参数等
  void onCmd(const std::string& topic, const std::string& payload);  // MQTT 回调入口：当上位机发送命令（JSON 格式）时触
  void spinOnce(double dt);// 主循环：更新估计、计算控制量、发布状态
  const RobotState& state() const { return state_; }  // 获取当前机器人状态（只读）
};  
} // namespace agv
