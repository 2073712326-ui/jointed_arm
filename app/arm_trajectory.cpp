#include "arm_trajectory.h"

bool ArmTrajectory::start(const CartesianReference& start,const CartesianReference& end, float duration_s) {
  if (running_ == true) {
    return false;
  }
  if (duration_s <= 0.f) {
    return false;
  }
  if (!ArmKinematics::lineReachable(start , end)){
    return false;
  }
  start_ = start;
  end_ = end;
  duration_s_ = duration_s;
  running_ = true;
  return true;
}
//把已经完成的单轴插值组合
CartesianReference ArmTrajectory::sample(float elapsed_s) {
  //初始化为零
  CartesianReference reference{};
  //未运行时，返回终点位置
  if (running_ == false) {
    reference.x = end_.x;
    reference.y = end_.y;
    reference.dx = reference.dy = reference.ddx = reference.ddy = 0.0f;
    return reference;
  }

  ArmTrajectory::interpolate(start_.x, end_.x,elapsed_s,duration_s_,reference.x,reference.dx,reference.ddx);
  ArmTrajectory::interpolate(start_.y,end_.y,elapsed_s,duration_s_,reference.y,reference.dy,reference.ddy);

  if (elapsed_s >= duration_s_) {
    running_ = false;
  }
  //返回本次采样得到的位置、速度和加速度
  return reference;

}
//单轴五次插值，位置坐标或关节角通用
void ArmTrajectory::interpolate(float start, float end, float elapsed_s,
                          float duration_s, float& position, float& velocity,
                          float& acceleration) {

  position = start;
  velocity = acceleration = 0.f;
  if (duration_s <= 0.f) {
    return;
  }
  if (elapsed_s <= 0.f) {
    return;
  }
  if (duration_s <= elapsed_s) {
    position = end;
    return;
  }
  if ( elapsed_s < duration_s) {
    const float u = elapsed_s / duration_s;

    const float u2 = u * u;
    const float u3 = u2 * u;
    const float u4 = u3 * u;
    const float u5 = u4 * u;
    const float s = 10 * u3 - 15 * u4 + 6 * u5;
    position = start + (end - start) * s;

    velocity = (end - start) / duration_s * (30 * u2 - 60 * u3 + 30 * u4);
    acceleration = (end - start) /(duration_s * duration_s) * (60 * u - 180 * u2 + 120 * u3);

  }
}

//主动取消当前轨迹状态
void ArmTrajectory::abort() {
  running_ = false;
}
