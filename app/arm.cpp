//
// Created by NoSignal on 2026/10/16.
//

#include "arm.h"
#include "base/common/math.h"
#include <cmath>

#include "stm32f4xx_hal.h"

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
//检查能否使能，把目标同步到当前位置，然后再让两个电机进入工作状态
bool Arm::enable() {
  if (calibrated_ == false) {
    return false;
  }
  if (!feedbackReady()) {
    return false;
  }
  if (mode_ == Mode::Joint) {
    return true;
  }

  updateFeedback();

  if (!withinLimits(state_.q)) {
    return false;
  }

  reference_.q[0] = state_.q[0];
  reference_.q[1] = state_.q[1];
  reference_.dq[0] = 0.0f;
  reference_.dq[1] = 0.0f;
  reference_.ddq[0] = 0.0f;
  reference_.ddq[1] = 0.0f;

  trajectory_.abort();//停止笛卡尔轨迹对象
  trajectory_type_ = TrajectoryType::None;//机械臂当前没有执行任何轨迹
  trajectory_finished_ = false;//清除完成标志

  //重置两个电机的PID历史数据
  joint0_.ppid_.reset();  // J0 位置环
  joint0_.spid_.reset();  // J0 速度环

  joint1_.ppid_.reset();  // J1 位置环
  joint1_.spid_.reset();  // J1 速度环

  //设置两个电机为POSITION_SPEED控制方式
  joint0_.method_ = Motor::POSITION_SPEED;
  joint1_.method_ = Motor::POSITION_SPEED;

  //先写入点击目标，角度转为deg，两个前馈均为零
  joint0_.setAngleSpeed(math::rad2deg(reference_.q[0]), 0.0f, 0.0f);
  joint1_.setAngleSpeed(math::rad2deg(reference_.q[1]), 0.0f, 0.0f);
  //进入工作状态
  joint0_.mode_ = Motor::WORKING;
  joint1_.mode_ = Motor::WORKING;
  mode_ = Mode::Joint;
  return true;
}

void Arm::disable() {
  mode_ = Mode::Disabled;
  trajectory_.abort();//停止笛卡尔轨迹对象
  trajectory_type_ = TrajectoryType::None;//机械臂当前没有执行任何轨迹
  trajectory_finished_ = false;

  reference_.dq[0] = 0.0f;
  reference_.dq[1] = 0.0f;
  reference_.ddq[0] = 0.0f;
  reference_.ddq[1] = 0.0f;
  //将两个电机切换为Motor::POWEROFF
  if (joint0_.mode_ != Motor::INIT ) {
    joint0_.mode_ = Motor::POWEROFF;
  }
  if (joint1_.mode_ != Motor::INIT ) {
    joint1_.mode_ = Motor::POWEROFF;
  }

  //清零两个电机的输出
  joint0_.intensity_ = 0;
  joint0_.intensity_float_ = 0;
  joint1_.intensity_ = 0.0f;
  joint1_.intensity_float_ = 0.0f;

  //清除两个电机的前馈
  joint0_.control_data_.feedforward_speed = 0.0f;
  joint0_.control_data_.feedforward_intensity = 0.0f;
  joint1_.control_data_.feedforward_intensity = 0.0f;
  joint1_.control_data_.feedforward_speed = 0.0f;

  //重置两个电机的PID
  joint0_.ppid_.reset();
  joint0_.spid_.reset();
  joint1_.ppid_.reset();
  joint1_.spid_.reset();
}

bool Arm::setReference(const Reference& reference) {
  if (trajRunning()) {
    return false;
  }
  return applyReference(reference);
}


bool Arm::applyReference(const Reference& reference) {
  if (mode_ != Mode::Joint) {
    return false;
  }
  if (!calibrated_) {
    return false;
  }
  if (!feedbackReady()) {
    return false;
  }
  if (!withinLimits(reference.q)) {
    return false;
  }
  reference_ = reference;

  return true;
}
//把手动输入的关节速度转换成目标角度
bool Arm::addJointReference(float speed0, float speed1, float dt_s) {
  if (trajRunning()) {
    return false;
  }
  if (dt_s <= 0.0f) {
    return false;
  }
  Reference next = reference_;
  next.q[0] = reference_.q[0] + speed0 * dt_s;
  next.q[1] = reference_.q[1] + speed1 * dt_s;
  //设置速度前馈
  next.dq[0] = speed0;
  next.dq[1] = speed1;
  //两个关节的加速度前馈清零
  next.ddq[0] = 0.0f;
  next.ddq[1] = 0.0f;

  for (int i = 0; i < 2; i++) {
    next.q[i] = math::limit(next.q[i],limits_.q_min[i], limits_.q_max[i]);
    if ((next.q[i] <= limits_.q_min[i] && next.dq[i] < 0.0f) || (next.q[i] >= limits_.q_max[i] && next.dq[i] > 0.0f)){
      next.dq[i] = 0.0f;
    }
  }
  //将计算好的目标保存到机械臂中
  return setReference(next);
}
//检查机械臂当前是否允许启动新轨迹
bool Arm::trajectoryReady() {
  if (mode_ != Mode::Joint) {
    return false;
  }
  if (!calibrated_) {
    return false;
  }
  if (!feedbackReady()) {
    return false;
  }
  if (trajRunning()) {
    return false;
  }

  updateFeedback();
  if (!withinLimits(state_.q)) {
    return false;
  }
  if (!withinLimits(reference_.q)) {
    return false;
  }
  return true;
}

//启动一段末端直线轨迹，每个控制周期的位置更新
bool Arm::trajStart(float x_m, float y_m, float duration_s) {
  if (duration_s <= 0.0f) {
    return false;
  }
  if (!trajectoryReady()) {
    return false;
  }
  //用当前目标关节角计算末端起点
  CartesianReference start = ArmKinematics::forward(reference_.q);

  //建立末端终点
  CartesianReference end{};
  end.x = x_m;
  end.y = y_m;
  //根据当前第二关节的相对角，选择相同的逆解分支
  int elbow;
  //reference_.q[1]是第二根连杆相对于第一根连杆的目标角度
  if (std::sin(reference_.q[1]) >= 0.0f) {
    elbow = 1;
  } else {
    elbow = -1;
  }

  //对终点求逆解，并检查终点关节角限位
  float end_q[2]{};
  if (!ArmKinematics::inverse(end, reference_.q, elbow, end_q)) {
    return false;
  }
  if (!withinLimits(end_q)) {
    return false;
  }
  if (!trajectory_.start(start, end, duration_s)) {
    return false;
  }
  //启动成功后，保存轨迹状态
  trajectory_elbow_ = elbow;
  trajectory_start_tick_ = HAL_GetTick();
  trajectory_finished_ = false;
  trajectory_type_ = TrajectoryType::Cartesian;

  return true;
}

bool Arm::trajStartJoint(float q0_rad, float q1_rad, float duration_s) {
  if (duration_s <= 0.0f) {
    return false;
  }
  if (!trajectoryReady()) {
    return false;
  }
  //将两个目标角度组成数组
  const float target_q[2] = { q0_rad, q1_rad };
  if (!withinLimits(target_q)) {
    return false;
  }
  //保存两个关节的起点和终点以及运动时间
  joint_trajectory_start_[0] = state_.q[0];
  joint_trajectory_start_[1] = state_.q[1];
  joint_trajectory_end_[0] = target_q[0];
  joint_trajectory_end_[1] = target_q[1];
  joint_trajectory_duration_ = duration_s;

  reference_.q[0] = state_.q[0];
  reference_.q[1] = state_.q[1];
  reference_.dq[0] = 0.0f;
  reference_.dq[1] = 0.0f;
  reference_.ddq[0] = 0.0f;
  reference_.ddq[1] = 0.0f;

  //保存启动状态
  trajectory_type_ = TrajectoryType::Joint;
  trajectory_start_tick_ = HAL_GetTick();
  trajectory_finished_ = false;

  return true;
}
//取消轨迹，并保持最后一次输出的目标角度
//不调用disable() 取消轨迹后，电机继续保持使能
void Arm::trajAbort() {
  trajectory_ .abort();
  //机械臂不再执行任何轨迹
  trajectory_type_ = TrajectoryType::None;
  trajectory_finished_ = false;

  reference_.dq[0] = 0.0f;
  reference_.dq[1] = 0.0f;
  reference_.ddq[0] = 0.0f;
  reference_.ddq[1] = 0.0f;

}

// 根据经过时间，更新关节轨迹或末端直线轨迹的参考
void Arm::trajectoryPlanner() {
  if (!trajRunning()) {
    return;
  }
  //将启动到现在经过的时间从毫秒转换为秒
  const float elapsed_s =(HAL_GetTick() - trajectory_start_tick_) * 0.001f;

  Reference next{};
  if (trajectory_type_ == TrajectoryType::Joint) {
    //每个关节调用一次ArmTrajectory::interpolate()
    ArmTrajectory::interpolate(joint_trajectory_start_[0],joint_trajectory_end_[0],elapsed_s,
  joint_trajectory_duration_,next.q[0],next.dq[0],next.ddq[0]);
    ArmTrajectory::interpolate(joint_trajectory_start_[1],joint_trajectory_end_[1],elapsed_s,
    joint_trajectory_duration_,next.q[1],next.dq[1],next.ddq[1]);


    if (!applyReference(next)) {
    trajAbort();
    return;
    }
    if (elapsed_s >= joint_trajectory_duration_) {
    trajectory_type_ = TrajectoryType::None;
    trajectory_finished_ = true;
    }
    return;
  }
  //直线轨迹分支
  if (trajectory_type_ == TrajectoryType::Cartesian) {
    //采样当前时刻的末端目标
    CartesianReference position = trajectory_.sample(elapsed_s);
    //对本周期的末端目标求逆解
    if (!ArmKinematics::inverse(position,reference_.q,trajectory_elbow_,next.q)){
      trajAbort();
      return;
    }
    if (!applyReference(next)) {
      trajAbort();
      return;
    }
    if (!trajectory_.running()) {
      trajectory_type_ = TrajectoryType::None;
      trajectory_finished_ = true;
    }
    return;
  }
}

//把两个电机的反馈转换为机械臂状态
void Arm::updateFeedback() {
  state_.q[0] = math::deg2rad(joint0_.realAngle());
  state_.q[1] = math::deg2rad(joint1_.realAngle());
  state_.dq[0] = math::dps2radps(joint0_.realSpeed());
  state_.dq[1] = math::dps2radps(joint1_.realSpeed());
}

void Arm::handle() {
  if (!feedbackReady()) {
    calibrated_ = false;
    disable();
    return;
  }
  updateFeedback();

  if (mode_ == Mode::Disabled) {
    disable();
    return;
  }

  if (!calibrated_) {
    disable();
    return;
  }

  if (!withinLimits(state_.q) || !withinLimits(reference_.q)) {
    disable();
    return;
  }
  trajectoryPlanner();//推进轨迹
  writeMotorCommands();//写入电机目标
}

//把机械臂参考转换为电机接口需要的单位
//只更新电机控制目标，后续电机控制与CAN通信流程负责计算、发送实际输出
void Arm::writeMotorCommands() {
  if (mode_ != Mode::Joint) {
    return;
  }
  //计算两个关节的加速度输出前馈
  float ff0 = feedforward_.k_acc[0] * reference_.ddq[0];
  float ff1 = feedforward_.k_acc[1] * reference_.ddq[1];

  joint0_.setAngleSpeed(math::rad2deg(reference_.q[0]),math::radps2dps(reference_.dq[0]),ff0);
  joint1_.setAngleSpeed(math::rad2deg(reference_.q[1]),math::radps2dps(reference_.dq[1]),ff1);

}
