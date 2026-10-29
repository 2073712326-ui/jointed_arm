//
// Created by NoSignal on 2026/10/16.
//

#ifndef DOUBLE_JOINTED_ROBOTIC_ARM_ARM_H
#define DOUBLE_JOINTED_ROBOTIC_ARM_ARM_H

#include "base/motor/motor.h"
#include "arm_trajectory.h"

// 双关节机械臂
class Arm {
 public:
  struct State {
    float q[2]{};   // rad，J1 为相对前一连杆的角度
    float dq[2]{};  // 实际关节速度（rad/s）
  };

  struct Reference {
    float q[2]{};    // rad，目标位置
    float dq[2]{};   // rad/s，速度前馈
    float ddq[2]{};  // rad/s²，加速度前馈
  };

  //保存两个关节的角度上下限
  struct Limits {
    float q_min[2];  // rad
    float q_max[2];  // rad
  };
  //表示把关节目标加速度转换成输出指令的系数
  struct Feedforward {
    float k_acc[2];  // 输出指令 / (rad/s²)
  };

  enum class Mode { Disabled, Joint };//表示机械臂是否处于允许关节控制的状态

  Arm(Motor& joint0, Motor& joint1, const Limits& limits,
      const Feedforward& feedforward);

  // 仅失能且反馈有效时按已知姿态标定
  //建立软件角度和机械姿态的对应关系
  bool calibrate(float q0_rad, float q1_rad);//按已知机械姿态建立关节角度零点
  // 以当前位置为目标使能
  bool enable();//以当前位置作为初始目标，进入可控制状态
  void disable();//停止动作并使两个关节失能
  // 轨迹中拒绝外部参考；越限返回 false
  bool setReference(const Reference& reference);
  // 手动关节速度输入，内部积分并处理限位；单位 rad/s、s
  bool addJointReference(float speed0, float speed1, float dt_s = 0.001f);
  // 从当前规划目标起步，沿用其肘支和累计圈数；单位 m、s
  bool trajStart(float x_m, float y_m, float duration_s);
  // 从实际反馈开始的关节空间同步点到点，目标为累计角度 rad
  bool trajStartJoint(float q0_rad, float q1_rad, float duration_s);
  // 停止推进并保持位置参考，清除速度和加速度前馈
  void trajAbort();
  bool trajRunning() const { return trajectory_type_ != TrajectoryType::None; }
  // 正常输出终点后为 true；取消或新轨迹启动后为 false
  bool trajFinished() const { return trajectory_finished_; }
  // 1 ms 调用一次，在 allMotorsHandle() 之前
  void handle();

  const State& state() const { return state_; }
  const Reference& reference() const { return reference_; }
  Mode mode() const { return mode_; }
  bool calibrated() const { return calibrated_; }

 private:
  enum class TrajectoryType { None, Cartesian, Joint };//表示当前是否正在执行轨迹，以及轨迹属于哪种空间
  bool feedbackReady();//检查两电机反馈是否可用于控制
  bool trajectoryReady();//检查能否开始轨迹
  bool withinLimits(const float q[2]) const;//检查两个关节角是否都在限位内
  bool applyReference(const Reference& reference);
  void updateFeedback();//从Motor获取反馈并转换到Arm单位
  void trajectoryPlanner();//根据经过时间生成本周期轨迹参考
  void writeMotorCommands();//把参考写给两个Motor

  Motor& joint0_;
  Motor& joint1_;
  const Limits limits_;
  const Feedforward feedforward_;
  State state_{};
  Reference reference_{};
  Mode mode_{Mode::Disabled};
  bool calibrated_{false};//按已知机械姿态建立关节角度零点
  ArmTrajectory trajectory_{};
  TrajectoryType trajectory_type_{TrajectoryType::None};
  bool trajectory_finished_{false};
  uint32_t trajectory_start_tick_{};
  int trajectory_elbow_{1};
  float joint_trajectory_start_[2]{};
  float joint_trajectory_end_[2]{};
  float joint_trajectory_duration_{};
};

#endif  // DOUBLE_JOINTED_ROBOTIC_ARM_ARM_H
