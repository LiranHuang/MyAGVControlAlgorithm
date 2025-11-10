#pragma once
namespace agv {
class Kalman1D {   // 一维卡尔曼滤波器类（用于单变量估计，例如位置x或角度yaw）
private:
  double x_{0},  // 状态
  p_{1},   //协方差
  q_{1e-3},  //过程噪声
  r_{1e-2};  //测量噪声
public:
  void setNoise(double q, double r){ q_ = q; r_ = r; }  // 设置滤波器的噪声参数
  void init(double x0, double p0){ x_ = x0; p_ = p0; }  // 初始化状态：设定初始估计值 x0 与初始协方差 p0
  void predict(double u=0){ 
    x_ += u;  // 状态预测：假设系统每步状态变化为控制输入 u
    p_ += q_;  // 协方差增加过程噪声
  }  
  double update(double z){  // 输入：传感器测量值 z
    double k = p_ / (p_ + r_);  // 卡尔曼增益 k = 预测协方差 / (预测协方差 + 测量噪声)
    x_ += k * (z - x_);  // 状态更新：新的估计 = 预测值 + k * (测量误差)
    p_ = (1 - k) * p_;  // 更新协方差：减少不确定度
    return x_;
  }
  double x() const { return x_; }  // 返回当前状态估计（只读函数，const 表示不会修改内部变量）
};
} // namespace agv
