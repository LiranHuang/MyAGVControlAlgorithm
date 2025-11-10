#pragma once
#include <functional>   // 用于回调函数类型
#include <string>       // 用于字符串处理
#include <vector>       // 用于存储订阅主题列表等

namespace agv {  // 定义命名空间 agv，避免命名冲突

using MessageHandler = std::function<void(const std::string& topic, const std::string& payload)>;
// using MessageHandler 类型别名，写好后可以直接用 MessageHandler
// std::function 是 C++ 标准库里的通用可调用包装器
// void 表示回调函数不返回任何值
// 定义一个消息回调函数类型（主题+消息内容）
// 回调函数callback function是“把函数当作参数传给另一个函数”，由对方（系统或库）在合适的时候自动调用

class IMqttClient {
public:
  virtual ~IMqttClient() = default;
  virtual bool connect() = 0;
  virtual void disconnect() = 0;
  virtual bool publish(const std::string& topic, const std::string& payload, int qos=0, bool retain=false) = 0;
  virtual bool subscribe(const std::string& topic, int qos, MessageHandler cb) = 0;
  // 它们都是纯虚函数pure virtual functions，由MqttPahoClient.hpp/cpp继承并实现
};
} // namespace agv
// IMqttClient 是一个“接口类”（Interface Class）
