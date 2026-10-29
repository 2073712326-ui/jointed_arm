#pragma once

#include <cstdint>

// 只替换硬件边界；被测 Arm 与运动学/插值均编译真实源文件。
class Motor {
 public:
  enum Mode { POWEROFF, STOP, INIT, WORKING };
  enum ControlMethod { POSITION_SPEED, POSITION, SPEED, TORQUE };
  struct Connection {
    bool online{true};
    bool check() { return online; }
  } connect_;
  struct Controller {
    void reset() {}
  } ppid_, spid_;

  float& realAngle() { return angle; }
  float& realSpeed() { return speed; }
  void resetFeedbackAngle(float value) { angle = value; }
  void setAngleSpeed(float q, float dq, float ff) {
    target = q;
    feedforward_speed = dq;
    feedforward_output = ff;
  }

  Mode mode_{POWEROFF};
  ControlMethod method_{POSITION_SPEED};
  float intensity_float_{};
  int16_t intensity_{};
  float angle{}, speed{};
  float target{}, feedforward_speed{}, feedforward_output{};
};
