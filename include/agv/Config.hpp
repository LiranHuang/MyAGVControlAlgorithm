#pragma once
#include <string> // 引入 C++ 标准字符串类 std::string

// 定义配置结构体，用于保存 AGV 系统运行时参数
struct Config {
  std::string broker   = "tcp://localhost:1883";   // MQTT 代理服务器地址
  std::string clientId = "agv_node";               
  std::string topic_cmd = "agv/cmd";
  std::string topic_state = "agv/state";

  // PID 控制器参数（比例P，积分I，微分D）
  double pid_kp=1.0, pid_ki=0.0, pid_kd=0.1;
};

// 声明一个函数：从 JSON 文件加载配置数据
Config loadConfig(const std::string& json_path);
