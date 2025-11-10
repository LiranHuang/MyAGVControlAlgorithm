#include "agv/Config.hpp"
#include <nlohmann/json.hpp>
#include <fstream>
Config loadConfig(const std::string& path){
  Config c;
  std::ifstream f(path);
  if(!f.good()) return c;
  nlohmann::json j; f >> j;
  c.broker    = j.value("broker", c.broker);
  c.clientId  = j.value("clientId", c.clientId);
  c.topic_cmd   = j.value("topic_cmd", c.topic_cmd);
  c.topic_state = j.value("topic_state", c.topic_state);
  if(j.contains("pid")){
    c.pid_kp = j["pid"].value("kp", c.pid_kp);
    c.pid_ki = j["pid"].value("ki", c.pid_ki);
    c.pid_kd = j["pid"].value("kd", c.pid_kd);
  }
  return c;
}
