#include "agv/Executor.hpp"
#include "agv/Log.hpp"
#include <nlohmann/json.hpp>
using json = nlohmann::json;

namespace agv {
Executor::Executor(IMqttClient& mqtt, const Config& cfg)
  : mqtt_(mqtt), cfg_(cfg),
    pid_vx_(cfg.pid_kp, cfg.pid_ki, cfg.pid_kd),
    pid_wz_(cfg.pid_kp, cfg.pid_ki, cfg.pid_kd) {}

void Executor::init() {
  mqtt_.subscribe(cfg_.topic_cmd, 0,
    [this](const std::string& t, const std::string& p){ onCmd(t,p); });
  LOG_I("Executor initialized, subscribed to " << cfg_.topic_cmd);
}

void Executor::onCmd(const std::string&, const std::string& payload) {
  try {
    auto j = json::parse(payload);
    std::lock_guard<std::mutex> lk(mtx_);
    // 例：上位机发送 { "mode":"Executing", "target": { "vx":0.2, "wz":0.0 } }
    if(j.contains("mode")){
      std::string m = j["mode"];
      if(m=="Executing") state_.mode = Mode::Executing;
      else if(m=="Paused") state_.mode = Mode::Paused;
      else if(m=="EStop") state_.mode = Mode::EmergencyStop;
      else state_.mode = Mode::Idle;
    }
    if(j.contains("target")){
      state_.cmd.vx = j["target"].value("vx", 0.0);
      state_.cmd.wz = j["target"].value("wz", 0.0);
    }
  } catch(...) {
    LOG_W("Invalid JSON cmd: " << payload);
  }
}

void Executor::spinOnce(double dt) {
  std::lock_guard<std::mutex> lk(mtx_);
  // 简化：预测位姿（可接入里程计/KF）
  kf_x_.predict(state_.cmd.vx * dt);
  kf_yaw_.predict(state_.cmd.wz * dt);
  // 控制器（示例：跟踪0参考，可换成轨迹跟踪误差）
  double u_vx = pid_vx_.step(/*target*/0.0, /*current*/0.0);
  double u_wz = pid_wz_.step(/*target*/0.0, /*current*/0.0);
  // 发布状态
  json st{
    {"mode", (int)state_.mode},
    {"pose", {{"x", kf_x_.x()}, {"yaw", kf_yaw_.x()}}},
    {"cmd",  {{"vx", state_.cmd.vx}, {"wz", state_.cmd.wz}}},
    {"ctrl", {{"vx", u_vx}, {"wz", u_wz}}}
  };
  mqtt_.publish(cfg_.topic_state, st.dump(), 0, false);
}
} // namespace agv
