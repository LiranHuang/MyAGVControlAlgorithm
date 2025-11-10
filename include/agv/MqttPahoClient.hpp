#pragma once
#include "agv/IMqttClient.hpp"
#include <map>
#include <memory>

// 前向声明以避免头部引入重量级库
namespace mqtt { class async_client; class token; class delivery_token; class connect_options; }

namespace agv {
class MqttPahoClient : public IMqttClient {
  std::string broker_, client_id_;
  std::unique_ptr<mqtt::async_client> client_;
  std::unique_ptr<mqtt::connect_options> opts_;
  std::map<std::string, MessageHandler> handlers_;
public:
  MqttPahoClient(std::string broker, std::string client_id);
  bool connect() override;
  void disconnect() override;
  bool publish(const std::string& topic, const std::string& payload, int qos, bool retain) override;
  bool subscribe(const std::string& topic, int qos, MessageHandler cb) override;
};
} // namespace agv
