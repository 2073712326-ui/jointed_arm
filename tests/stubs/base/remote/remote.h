#pragma once
#include "base/motor/motor.h"
class RC {
 public:
  enum RCSwitchState_e { UP, MID, DOWN };
  struct RCSwitch { RCSwitchState_e l, r; } switch_{UP, DOWN};
  struct { int16_t l_col{}, r_col{}; } channel_;
  Motor::Connection connect_;
};
