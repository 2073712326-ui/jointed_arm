#include "arm_kinematics.h"
#include <cmath>
CartesianReference ArmKinematics::forward(const float q[2]) {
  CartesianReference position{};

  const float q1 = q[0];
  const float q2 = q[0] + q[1];

  position.x = l0 * std::cos(q1) + l1 * std::cos(q2);
  position.y = l0 * std::sin(q1) + l1 * std::sin(q2);

  return position;

}

bool ArmKinematics::inverse(const CartesianReference& position, const float previous_q[2] ,
                           int elbow, float q[2]) {
  const float x = position.x;
  const float y = position.y;
  const float r = x * x + y * y;

  float cos_q1 =  (r - l0 * l0 - l1 * l1) /( 2.0f * l0 * l1 );
  if (cos_q1 > 1.0f || cos_q1 < -1.0f) {
    return false;
  }
  if (cos_q1 > 1.0f) {
    cos_q1 = 1.0f;
  }else if (cos_q1 < -1.0f) {
    cos_q1 = -1.0f;
  }
  //q[1]的角度值
  float joint1 = elbow * std::acos(cos_q1);

  float alpha = std::atan2(y,x);
  float beta = std::atan2(l1 * std::sin(joint1),l0 + l1 * std::cos(joint1));
  //q[0]的角度值
  float joint0 = alpha - beta;

  const float two_pi = 6.28318530718f;


  joint0 += two_pi * std::round((previous_q[0] - joint0)/ two_pi);
  joint1 += two_pi * std::round((previous_q[1] - joint1)/ two_pi);

  q[0] = joint0;
  q[1] = joint1;
  return true;
}
//检查整条线段是否可以到达
bool ArmKinematics::lineReachable(const CartesianReference& start,const CartesianReference& end){
  //计算基座原点到线段的最大距离的平方
  const float start_p = start.x * start.x + start.y * start.y;
  const float end_p = end.x * end.x + end.y * end.y;
  float max_p;
  if (start_p > end_p) {
    max_p = start_p;
  }else {
    max_p = end_p;
  }

  //计算基座原点到线段的最小距离的平方
  const float dx = end.x - start.x;
  const float dy = end.y - start.y;
  const float d = dx * dx + dy * dy;

  float k = 0.0f;
  if (d > 0.0f){
    k = - (start.x * dx + start.y * dy) / d;
    if (k > 1.0f) {
      k = 1.0f;
    }else if (k < 0.0f) {
      k = 0.0f;
    }
  }
  const float closest_x = start.x + k * dx;
  const float closest_y = start.y + k * dy;
  const float min_p = closest_x * closest_x + closest_y * closest_y;

  const float cos_limit = std::sqrt(1.0f - min_sin_q1 * min_sin_q1);
  const float lower_r = l0 * l0 + l1 * l1 - 2 * l0 * l1 * cos_limit;
  const float upper_r = l0 * l0 + l1 * l1 + 2 * l0 * l1 * cos_limit;
  return min_p > lower_r && max_p < upper_r;

}



bool ArmKinematics::derivatives(const CartesianReference&, const float[2],
                               float[2], float[2]) {
  return false;
}
