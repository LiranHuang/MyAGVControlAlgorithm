#pragma once
#include <chrono>
namespace agv {
struct Pose2D { double x{0}, y{0}, yaw{0}; };
struct Twist2D { double vx{0}, wz{0}; };

enum class Mode { Idle, Executing, Paused, EmergencyStop };

struct RobotState {
  Pose2D  pose;
  Twist2D cmd;
  Mode    mode{Mode::Idle};
  std::chrono::steady_clock::time_point stamp{};
};
} // namespace agv
