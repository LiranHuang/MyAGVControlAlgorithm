#include "agv/MqttPahoClient.hpp"
#include <mqtt/async_client.h>

using namespace std::chrono_literals;
namespace agv {
MqttPahoClient::MqttPahoClient(std::string broker, std::string client_id)
  : broker_(std::move(broker)), client_id_(std::move(client_id)) {}

bool MqttPahoClient::connect() {
  client_ = std::make_unique<mqtt::async_client>(broker_, client_id_);
  mqtt::connect_options opts;
  try {
    client_->connect(opts)->wait();
    return true;
  } catch(const mqtt::exception& e) {
    return false;
  }
}
void MqttPahoClient::disconnect() {
  if(client_) client_->disconnect()->wait();
}
bool MqttPahoClient::publish(const std::string& topic, const std::string& payload, int qos, bool retain) {
  try {
    client_->publish(topic, payload.data(), payload.size(), qos, retain);
    return true;
  } catch(...) { return false; }
}
bool MqttPahoClient::subscribe(const std::string& topic, int qos, MessageHandler cb) {
  handlers_[topic] = std::move(cb);
  // 真实项目需绑定回调：set_callback(...)，在回调里查 handlers_ 并转发
  client_->subscribe(topic, qos)->wait();
  return true;
}
} // namespace agv
