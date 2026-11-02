//
// Created by NoSignal on 2026/10/16.
//

#include "arm.h"
#include "base/common/math.h"

Arm::Arm(Motor& joint0, Motor& joint1, const Limits& limits,
         const Feedforward& feedforward)
    : joint0_(joint0), joint1_(joint1), limits_(limits),
      feedforward_(feedforward) {}

bool Arm::feedbackReady() {
  //检查第一个电机是否在线
  if (!joint0_.connect_.check()) {
    return false;
  }
  if (!joint1_.connect_.check()) {//检查第二个电机是否在线
    return false;
  }
  if (joint0_.mode_ == Motor::INIT) {
    return false;
  }
  if (joint1_.mode_ == Motor::INIT) {//检查两个电机是否处于初始化状态
    return false;
  }
  return true;
}
//检查两个关节角是否在上下限内
bool Arm::withinLimits(const float q[2]) const {
  if (q[0] < limits_.q_min[0] || q[0] > limits_.q_max[0]) {
    return false;
  }
  if (q[1] < limits_.q_min[1] || q[1] > limits_.q_max[1]) {
    return false;
  }
  return true;
}

bool Arm::calibrate(float q0_rad, float q1_rad) {
  //机械臂必须处于失能状态
  if (mode_ != Mode::Disabled) {
    return false;
  }
  if (!feedbackReady()) {
    return false;
  }

  //检查传入的两个标定角度是否在限位内
  if (q0_rad < limits_.q_min[0] || q0_rad > limits_.q_max[0]) {
    return false;
  }
  if (q1_rad < limits_.q_min[1] || q1_rad > limits_.q_max[1]) {
    return false;
  }
  //将两个标定角度转换为deg
  joint0_.resetFeedbackAngle(math::rad2deg(q0_rad));
  joint1_.resetFeedbackAngle(math::rad2deg(q1_rad));

  //调用updateFeedback()，同步机械臂状态
  updateFeedback();
  //将位置参考同步到标定后的角度
  reference_.q[0] = state_.q[0];
  reference_.q[1] = state_.q[1];

  reference_.dq[0] = 0.0f;
  reference_.dq[1] = 0.0f;

  reference_.ddq[0] = 0.0f;
  reference_.ddq[1] = 0.0f;

  //设置calibrated_
  calibrated_ = true;
  return true;
}

bool Arm::enable() {
  return false;
}

void Arm::disable() {
}

bool Arm::setReference(const Reference&) {
  return false;
}

bool Arm::applyReference(const Reference&) {
  return false;
}

bool Arm::addJointReference(float, float, float) {
  return false;
}

bool Arm::trajectoryReady() {
  return false;
}

bool Arm::trajStart(float, float, float) {
  return false;
}

bool Arm::trajStartJoint(float, float, float) {
  return false;
}

void Arm::trajAbort() {
}

void Arm::trajectoryPlanner() {
}

//把两个电机的反馈转换为机械臂状态
void Arm::updateFeedback() {
  state_.q[0] = math::deg2rad(joint0_.realAngle());
  state_.q[1] = math::deg2rad(joint1_.realAngle());
  state_.dq[0] = math::dps2radps(joint0_.realSpeed());
  state_.dq[1] = math::dps2radps(joint1_.realSpeed());
}

void Arm::handle() {

}

void Arm::writeMotorCommands() {
}
