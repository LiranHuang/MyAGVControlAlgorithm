#include "agv/MqttPahoClient.hpp" // MQTT 客户端实现（基于 Paho C/C++）
#include "agv/Executor.hpp"       // 控制主循环/命令处理/状态发布
#include "agv/Config.hpp"         // 配置加载：broker、clientId、控制参数等
#include "agv/Log.hpp"            // 轻量日志封装 
#include <thread>                 // sleep_for / 线程工具
#include <chrono>                 // 高精度时间戳与间隔
 
int main(){
  Config cfg = loadConfig("config/default.json");
  // loadConfig读取config/default.json，得到MQTT 服务器地址 broker，客户端ID，指令主题 topic_cmd，状态主题 topic_state，PID 参数等

  agv::MqttPahoClient mqtt(cfg.broker, cfg.clientId);
  // 创建 MQTT 客户端，并尝试连接到 Broker
  // ::指明变量、函数或类属于哪个命名空间或类
  if(!mqtt.connect()){ 
    LOG_E("MQTT connect failed"); 
    return 1; 
  }
  // 连接到MQTT Broker失败则打印错误并退出

  agv::Executor exe(mqtt, cfg);
  // 构造执行器，把已经连上的mqtt和配置cfg交给Executor
  // agv::Executor 表示类 Executor 定义在命名空间 agv 中
  exe.init();
  // init() 内通常会：订阅命令主题、复位状态、加载 PID/Kalman 参数等

  using namespace std::chrono_literals;
  // 进入主循环：按照 ~50Hz 的频率调用 spinOnce(dt)
  auto last = std::chrono::steady_clock::now();
  // 使用单调时钟，记录时间起点

  while(true){
    auto now = std::chrono::steady_clock::now(); //当前时间
    double dt = std::chrono::duration<double>(now - last).count(); //时间差
    last = now; //更新“上一次时间”
    exe.spinOnce(dt);
    // 核心循环：每一帧/每一周期做一次控制和状态发布
    std::this_thread::sleep_for(20ms); // 50 Hz 控制循环频率
    // 节流至约 50Hz（20ms）。注意：如果 spinOnce 很重，可用“睡眠+补偿”或定时器。
  }
  return 0;
}
