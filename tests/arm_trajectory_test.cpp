#include "app/arm.h"
#include "app/control.h"
#include "base/common/math.h"
#include "base/remote/remote.h"
#include "main.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <initializer_list>

Motor control_m0, control_m1;
const Arm::Limits control_limits = {{-63.f, -63.f}, {63.f, 63.f}};
const Arm::Feedforward control_ff = {{0, 0}};
Arm arm(control_m0, control_m1, control_limits, control_ff);
RC rc;
TestUart test_uart;
int hiwdg = 0;
void robotControl();
void robotReset();
extern volatile bool arm_start_ok;
extern uint8_t arm_step;

namespace {
constexpr float pi = 3.14159265358979323846f;
uint32_t tick = 0;
int checks = 0;

#define CHECK(condition) do { \
  ++checks; \
  if (!(condition)) { \
    std::fprintf(stderr, "Line %d failed: %s\n", __LINE__, #condition); \
    std::exit(1); \
  } \
} while (0)

bool near(float a, float b, float tolerance = 2e-6f) {
  return std::fabs(a - b) <= tolerance;
}

CartesianReference point(float x, float y) {
  CartesianReference p;
  p.x = x;
  p.y = y;
  return p;
}

void zeroDerivatives(const Arm::Reference& ref) {
  for (int i = 0; i < 2; ++i) {
    CHECK(ref.dq[i] == 0.f);
    CHECK(ref.ddq[i] == 0.f);
  }
}

void kinematicsTests() {
  // 双分支、多圈输入的正逆解往返
  for (int branch : {-1, 1}) {
    for (int turn = -2; turn <= 2; ++turn) {
      for (int j = 1; j <= 3; ++j) {
        const float q[2] = {turn * 2.f * pi + 0.25f,
                            -turn * 2.f * pi + branch * j * 0.1f};
        const auto p = ArmKinematics::forward(q);
        float solved[2];
        CHECK(ArmKinematics::inverse(p, q, branch, solved));
        CHECK(near(solved[0], q[0], 6e-5f));
        CHECK(near(solved[1], q[1], 6e-5f));
        const auto recovered = ArmKinematics::forward(solved);
        CHECK(near(p.x, recovered.x));
        CHECK(near(p.y, recovered.y));
      }
    }
  }

  CHECK(ArmKinematics::lineReachable(point(.10f, .04f), point(.12f, .04f)));
  CHECK(ArmKinematics::lineReachable(point(.10f, .04f), point(.10f, .04f)));
  // 两端可达但穿过内孔、超出外径或经过伸直位
  CHECK(!ArmKinematics::lineReachable(point(-.10f, 0), point(.10f, 0)));
  CHECK(!ArmKinematics::lineReachable(point(.16f, 0), point(.12f, .04f)));
  CHECK(!ArmKinematics::lineReachable(point(.07f, 0), point(.12f, .04f)));
  CHECK(!ArmKinematics::lineReachable(point(.10f, .04f), point(.17f, 0)));

  float dq[2], ddq[2];
  const float singular_q[2] = {0, 0};
  CHECK(!ArmKinematics::derivatives(point(.16f, 0), singular_q, dq, ddq));

  // 用独立的解析关节运动核对雅可比和 J_dot 项
  for (int i = 1; i <= 6; ++i) {
    const float q[2] = {0.3f * i, (i % 2 ? -1.f : 1.f) * 1.2f};
    const float velocity[2] = {0.7f, -0.4f};
    const float acceleration[2] = {-0.2f, 0.5f};
    auto p = ArmKinematics::forward(q);
    const double a = q[0], b = q[0] + q[1];
    const double v = velocity[0], w = velocity[0] + velocity[1];
    const double acc = acceleration[0], acc01 = acceleration[0] + acceleration[1];
    p.dx = -.045 * std::sin(a) * v - .115 * std::sin(b) * w;
    p.dy = .045 * std::cos(a) * v + .115 * std::cos(b) * w;
    p.ddx = -.045 * (std::cos(a) * v * v + std::sin(a) * acc)
            -.115 * (std::cos(b) * w * w + std::sin(b) * acc01);
    p.ddy = .045 * (-std::sin(a) * v * v + std::cos(a) * acc)
            +.115 * (-std::sin(b) * w * w + std::cos(b) * acc01);
    CHECK(ArmKinematics::derivatives(p, q, dq, ddq));
    for (int j = 0; j < 2; ++j) {
      CHECK(near(dq[j], velocity[j], 3e-5f));
      CHECK(near(ddq[j], acceleration[j], 3e-5f));
    }
  }
}

void trajectoryTests() {
  ArmTrajectory trajectory;
  const auto start = point(.12f, -.02f), end = point(.12f, .02f);
  CHECK(!trajectory.start(start, end, 0));
  CHECK(trajectory.start(start, end, 2));
  CHECK(!trajectory.start(start, point(.1f, .06f), 1));  // 运行中
  auto p = trajectory.sample(0);
  CHECK(p.x == start.x && p.y == start.y);
  CHECK(p.dx == 0 && p.dy == 0 && p.ddx == 0 && p.ddy == 0);
  float last_y = start.y;
  for (int ms = 0; ms <= 2000; ++ms) {
    p = trajectory.sample(ms * .001f);
    if (ms % 100 == 0) {
      CHECK(near(p.x, .12f));
      CHECK(p.y + 2e-6f >= last_y && p.y <= end.y + 2e-6f);
      CHECK(p.dx == 0 && p.ddx == 0);
      last_y = p.y;
    }
  }
  CHECK(!trajectory.running());
  CHECK(p.x == end.x && p.y == end.y);
  CHECK(p.dx == 0 && p.dy == 0 && p.ddx == 0 && p.ddy == 0);

  // 穿过 atan2 的负 X 轴切口时关节参考连续，并保留多圈
  const auto a = point(-.12f, .02f), b = point(-.12f, -.02f);
  for (int branch : {-1, 1}) {
    float seed[2] = {8.f * pi, branch * 1.3f + 4.f * pi};
    float previous[2];
    CHECK(ArmKinematics::inverse(a, seed, branch, previous));
    CHECK(trajectory.start(a, b, 2));
    for (int ms = 0; ms <= 2000; ++ms) {
      p = trajectory.sample(ms * .001f);
      if (ms % 10 == 0) {
        float q[2], dq[2], ddq[2];
        CHECK(ArmKinematics::inverse(p, previous, branch, q));
        CHECK(ArmKinematics::derivatives(p, q, dq, ddq));
        CHECK(std::sin(q[1]) * branch > 0);
        CHECK(std::fabs(q[0] - previous[0]) < .01f);
        CHECK(std::fabs(q[1] - previous[1]) < .01f);
        const auto recovered = ArmKinematics::forward(q);
        CHECK(near(recovered.x, p.x));
        CHECK(near(recovered.y, p.y));
        previous[0] = q[0];
        previous[1] = q[1];
      }
    }
    CHECK(previous[0] > 6.f * pi && previous[1] > 2.f * pi);
  }

  // 中心差分核对插值的一、二阶导数
  CHECK(trajectory.start(start, end, 2));
  for (float t : {.7f, 1.3f}) {
    const float h = .005f;
    const auto left = trajectory.sample(t - h);
    const auto center = trajectory.sample(t);
    const auto right = trajectory.sample(t + h);
    CHECK(near((right.y - left.y) / (2 * h), center.dy, 3e-6f));
    CHECK(near((right.dy - left.dy) / (2 * h), center.ddy, 5e-6f));
  }
}

void armTests() {
  Motor m0, m1;
  const Arm::Limits limits = {{-20 * pi, -20 * pi}, {20 * pi, 20 * pi}};
  const Arm::Feedforward ff = {{0, 0}};
  Arm arm(m0, m1, limits, ff);
  CHECK(!arm.trajStart(.12f, .04f, 3));  // 未标定
  CHECK(arm.calibrate(0, 0));
  CHECK(arm.enable());
  CHECK(!arm.trajStart(.12f, .04f, 3));  // 伸直起步被拒绝
  arm.disable();
  float seed[2] = {0, 1}, initial_q[2];
  CHECK(ArmKinematics::inverse(point(.1f, .04f), seed, 1, initial_q));
  CHECK(arm.calibrate(initial_q[0], initial_q[1]));
  CHECK(arm.enable());
  const auto old = arm.reference();
  CHECK(!arm.trajStart(-.1f, -.04f, 3));  // 穿过内孔
  CHECK(!arm.trajStart(.12f, .04f, 0));
  CHECK(!arm.trajRunning());
  CHECK(arm.reference().q[0] == old.q[0]);
  auto moving = old;
  moving.dq[0] = .1f;
  CHECK(arm.setReference(moving));
  CHECK(arm.reference().dq[0] == .1f);  // setReference 不再清前馈
  arm.trajAbort();
  zeroDerivatives(arm.reference());
  CHECK(arm.setReference(old));
  tick = 0;
  CHECK(arm.trajStart(.12f, .04f, 3));
  CHECK(!arm.setReference(old));  // 轨迹中拒绝外部参考
  for (int ms = 0; ms <= 3000; ++ms) {
    tick = ms;
    arm.handle();
    if (ms % 100 != 0) continue;
    const auto p = ArmKinematics::forward(arm.reference().q);
    CHECK(near(p.y, .04f));
    CHECK(p.x >= .1f - 2e-6f && p.x <= .12f + 2e-6f);
    CHECK(near(m0.target, math::rad2deg(arm.reference().q[0])));
    CHECK(near(m1.feedforward_speed, math::rad2deg(arm.reference().dq[1])));
    CHECK(m0.feedforward_output == 0 && m1.feedforward_output == 0);
  }
  CHECK(!arm.trajRunning());
  auto finished = ArmKinematics::forward(arm.reference().q);
  CHECK(near(finished.x, .12f) && near(finished.y, .04f));
  zeroDerivatives(arm.reference());

  // 取消后保持最后的位置参考
  tick = 0;
  CHECK(arm.trajStart(.12f, .04f, 3));
  tick += 1000;
  arm.handle();
  const auto held = arm.reference();
  arm.trajAbort();
  CHECK(!arm.trajRunning());
  zeroDerivatives(arm.reference());
  tick += 1000;
  arm.handle();
  CHECK(arm.reference().q[0] == held.q[0]);
  CHECK(arm.setReference(held));
  arm.trajAbort();

  CHECK(arm.trajStart(.12f, .04f, 3));
  arm.disable();
  CHECK(!arm.trajRunning() && arm.mode() == Arm::Mode::Disabled);
  CHECK(m0.mode_ == Motor::POWEROFF && m0.intensity_ == 0);
  zeroDerivatives(arm.reference());
  CHECK(arm.enable());
  CHECK(arm.trajStart(.12f, .04f, 3));
  m1.connect_.online = false;
  arm.handle();
  CHECK(!arm.trajRunning() && !arm.calibrated());
  CHECK(arm.mode() == Arm::Mode::Disabled);
  m1.connect_.online = true;
  CHECK(!arm.enable());
  CHECK(arm.calibrate(initial_q[0], initial_q[1]));
  CHECK(arm.enable());
  CHECK(!arm.trajRunning());

  // 毫秒计数回绕后仍能结束
  tick = 0xFFFFFFFFu - 999u;
  CHECK(arm.trajStart(.12f, .04f, 3));
  tick += 3000;
  arm.handle();
  CHECK(!arm.trajRunning());
  zeroDerivatives(arm.reference());

  // 运行中越过已有角度限位：不发布越界样本并取消轨迹
  const Arm::Limits tight = {{initial_q[0] - .01f, initial_q[1] - .01f},
                            {initial_q[0] + .01f, initial_q[1] + .01f}};
  Arm limited(m0, m1, tight, ff);
  CHECK(limited.calibrate(initial_q[0], initial_q[1]));
  CHECK(limited.enable());
  tick = 0;
  CHECK(limited.trajStart(.12f, .04f, 3));
  tick += 1500;
  limited.handle();
  CHECK(!limited.trajRunning());
  CHECK(near(limited.reference().q[0], initial_q[0]));
  zeroDerivatives(limited.reference());
}

void plannedStartTests() {
  Motor m0, m1;
  Arm planned(m0, m1, control_limits, control_ff);
  // 反馈的肘支、圈数不动摇直线规划起点
  for (int branch : {-1, 1}) {
    planned.disable();
    CHECK(planned.calibrate(4 * pi, 6 * pi + branch * pi / 2));
    CHECK(planned.enable());
    const auto start = planned.reference();
    for (float feedback_q1 : {0.f, -branch * pi / 2}) {
      CHECK(planned.setReference(start));
      m0.angle = 0;
      m1.angle = math::rad2deg(feedback_q1);
      planned.handle();
      CHECK(planned.trajStart(.085f, branch * .115f, 1.f));
      CHECK(planned.reference().q[0] == start.q[0]);
      CHECK(planned.reference().q[1] == start.q[1]);
      planned.handle();
      for (int ms = 1; ms <= 1000; ++ms) {
        ++tick;
        planned.handle();
        if (ms % 100 != 0) continue;
        CHECK(std::sin(planned.reference().q[1]) * branch > 0);
        CHECK(std::fabs(planned.reference().q[0] - start.q[0]) < pi);
        CHECK(std::fabs(planned.reference().q[1] - start.q[1]) < pi);
        if (ms == 500) {
          const auto midpoint = ArmKinematics::forward(planned.reference().q);
          CHECK(near(midpoint.x, .065f));
          CHECK(near(midpoint.y, branch * .115f));
        }
      }
      CHECK(!planned.trajRunning() && planned.trajFinished());
      const auto end = ArmKinematics::forward(planned.reference().q);
      CHECK(near(end.x, .085f) && near(end.y, branch * .115f));
      zeroDerivatives(planned.reference());
    }
  }
}

void jointTrajectoryTests() {
  Motor m0, m1;
  const Arm::Limits limits = {{-20 * pi, -20 * pi}, {20 * pi, 20 * pi}};
  const Arm::Feedforward ff = {{0, 0}};
  Arm joint_arm(m0, m1, limits, ff);
  CHECK(!joint_arm.trajStartJoint(0, pi / 2, 2));
  CHECK(joint_arm.calibrate(0, 0));
  CHECK(joint_arm.enable());
  CHECK(!joint_arm.trajStartJoint(0, pi / 2, 0));
  CHECK(!joint_arm.trajStartJoint(0, 21 * pi, 2));  // 目标越限
  tick = 0;
  CHECK(joint_arm.trajStartJoint(0, pi / 2, 2));
  CHECK(!joint_arm.trajFinished());
  CHECK(!joint_arm.addJointReference(0, 1));  // 轨迹中拒绝手动
  CHECK(!joint_arm.trajStartJoint(0, pi, 2));  // 已有轨迹
  tick = 1000;
  joint_arm.handle();
  CHECK(near(joint_arm.reference().q[1], pi / 4));
  tick = 2000;
  joint_arm.handle();
  CHECK(!joint_arm.trajRunning() && joint_arm.trajFinished());
  CHECK(joint_arm.reference().q[0] == 0);
  CHECK(joint_arm.reference().q[1] == pi / 2);
  zeroDerivatives(joint_arm.reference());
  joint_arm.trajAbort();
  CHECK(!joint_arm.trajFinished());

  // 手动速度积分并在限位处清掉向外速度
  const float before = joint_arm.reference().q[0];
  CHECK(joint_arm.addJointReference(.5f, -.2f, .01f));
  CHECK(near(joint_arm.reference().q[0], before + .005f));
  CHECK(joint_arm.reference().dq[1] == -.2f);
  auto ref = joint_arm.reference();
  ref.q[0] = limits.q_max[0];
  CHECK(joint_arm.setReference(ref));
  CHECK(joint_arm.addJointReference(1, -.2f));
  CHECK(joint_arm.reference().q[0] == limits.q_max[0]);
  CHECK(joint_arm.reference().dq[0] == 0);
  joint_arm.disable();
  CHECK(!joint_arm.trajFinished());
  CHECK(!joint_arm.addJointReference(1, 0));
}

// 理想一周期跟踪，仅用于验证控制任务衔接
void controlCycle(bool follow_reference = true) {
  if (follow_reference) {
    control_m0.angle = math::rad2deg(arm.reference().q[0]);
    control_m1.angle = math::rad2deg(arm.reference().q[1]);
    control_m0.speed = math::rad2deg(arm.reference().dq[0]);
    control_m1.speed = math::rad2deg(arm.reference().dq[1]);
  }
  ++tick;
  robotControl();
  arm.handle();
}

// 调用时预备参考已完成，逐毫秒检查每条边的几何路径与完成时刻
void squareEdges(bool lagging_feedback = false) {
  const float vertices[5][2] = {
      {.100f, .100f}, {.100f, -.100f}, {-.100f, -.100f},
      {-.100f, .100f}, {.100f, .100f}};
  const auto initial = arm.reference();
  for (int edge = 0; edge < 4; ++edge) {
    CHECK(arm_step == edge + 1 && arm.trajFinished());
    const auto start = point(vertices[edge][0], vertices[edge][1]);
    if (lagging_feedback) {
      // 启动前注入未到顶点的反馈，模拟跟踪滞后
      const auto feedback = point(start.x - .003f, start.y - .002f);
      float q[2];
      CHECK(ArmKinematics::inverse(feedback, arm.reference().q, 1, q));
      control_m0.angle = math::rad2deg(q[0]);
      control_m1.angle = math::rad2deg(q[1]);
    }
    const auto previous = arm.reference();
    controlCycle(!lagging_feedback);
    CHECK(arm_start_ok && arm.trajRunning() && !arm.trajFinished());
    CHECK(arm_step == edge + 2);
    auto p = ArmKinematics::forward(arm.reference().q);
    CHECK(near(p.x, start.x) && near(p.y, start.y));
    for (int j = 0; j < 2; ++j) {
      CHECK(near(arm.reference().q[j], previous.q[j], 2e-5f));
    }
    const auto end = point(vertices[edge + 1][0], vertices[edge + 1][1]);
    CHECK(ArmKinematics::lineReachable(start, end));
    for (int ms = 1; ms <= 1000; ++ms) {
      controlCycle(!lagging_feedback);
      if (ms % 100 != 0 && ms != 1000) continue;
      CHECK(arm.trajRunning() == (ms < 1000));
      CHECK(arm.trajFinished() == (ms == 1000));
      CHECK(arm_step == edge + 2);
      p = ArmKinematics::forward(arm.reference().q);
      const float u = ms / 1000.f;
      const float s = u * u * u * (10.f + u * (-15.f + 6.f * u));
      CHECK(near(p.x, start.x + s * (end.x - start.x), 5e-6f));
      CHECK(near(p.y, start.y + s * (end.y - start.y), 5e-6f));
      CHECK(std::sin(arm.reference().q[1]) > ArmKinematics::min_sin_q1);
    }
    CHECK(near(p.x, end.x) && near(p.y, end.y));
    CHECK(near(arm.reference().q[0], initial.q[0] - (edge + 1) * pi / 2, 2e-5f));
    CHECK(near(arm.reference().q[1], initial.q[1], 2e-5f));
    zeroDerivatives(arm.reference());
  }
  const auto finished = arm.reference();
  CHECK(near(finished.q[0] - initial.q[0], -2 * pi, 2e-5f));
  for (int i = 0; i < 1000; ++i) controlCycle(!lagging_feedback);
  CHECK(arm_step == 0 && !arm.trajRunning() && arm.trajFinished());
  CHECK(arm.reference().q[0] == finished.q[0]);
  CHECK(arm.reference().q[1] == finished.q[1]);
}

void controlTests() {
  tick = 0;
  controlInit();
  robotReset();
  // 未标定时拒绝自动动作
  CHECK(!arm.calibrated());
  rc.switch_ = {RC::MID, RC::UP};
  controlCycle();
  CHECK(!arm_start_ok && !arm.trajRunning());

  // 左中右中失能并校准伸直零位
  rc.switch_ = {RC::MID, RC::MID};
  controlCycle();
  CHECK(arm.calibrated() && arm.mode() == Arm::Mode::Disabled);
  CHECK(arm.reference().q[0] == 0 && arm.reference().q[1] == 0);
  CHECK(!arm.trajRunning());

  // 2 s 预备姿态，从实际反馈起步
  rc.switch_ = {RC::MID, RC::UP};
  controlCycle();
  CHECK(arm_start_ok && arm.trajRunning() && arm_step == 1);
  CHECK(arm.mode() == Arm::Mode::Joint);
  for (int i = 0; i < 2000; ++i) controlCycle();
  CHECK(!arm.trajRunning() && arm.trajFinished());
  auto prep = ArmKinematics::forward(arm.reference().q);
  CHECK(near(prep.x, .1f) && near(prep.y, .1f));
  zeroDerivatives(arm.reference());
  rc.channel_.l_col = 500;  // 自动动作期间摇杆不覆盖参考
  squareEdges();

  // 手动接管，摇杆回中后保持
  rc.switch_ = {RC::UP, RC::UP};
  rc.channel_.l_col = 0;
  controlCycle();
  CHECK(arm.mode() == Arm::Mode::Joint);
  const auto before_manual = arm.reference();
  rc.channel_.l_col = 200;
  controlCycle();
  CHECK(near(arm.reference().q[0] - before_manual.q[0],
             math::deg2rad(1440.f) * (180.f / 640.f) * .001f));
  CHECK(arm.reference().q[1] == before_manual.q[1]);
  rc.channel_.l_col = 0;
  controlCycle();

  const auto before_deadband = arm.reference();
  rc.channel_.l_col = 20;
  rc.channel_.r_col = -20;
  controlCycle();
  CHECK(arm.reference().q[0] == before_deadband.q[0]);
  CHECK(arm.reference().q[1] == before_deadband.q[1]);
  rc.channel_.l_col = -660;
  rc.channel_.r_col = 660;
  controlCycle();
  CHECK(near(arm.reference().q[0] - before_deadband.q[0],
             math::deg2rad(-1440.f) * .001f));
  CHECK(near(arm.reference().q[1] - before_deadband.q[1],
             math::deg2rad(1440.f) * .001f));
  rc.channel_.l_col = rc.channel_.r_col = 0;
  controlCycle();

  // 其它挡位只保持目标
  rc.switch_ = {RC::UP, RC::MID};
  const auto held = arm.reference();
  controlCycle();
  CHECK(arm.mode() == Arm::Mode::Joint);
  CHECK(arm.reference().q[1] == held.q[1]);

  // 预备中取消后不继续走方框
  rc.switch_ = {RC::MID, RC::UP};
  controlCycle();
  CHECK(arm_start_ok && arm.trajRunning());
  arm.trajAbort();
  for (int i = 0; i < 10; ++i) controlCycle();
  CHECK(!arm_start_ok && !arm.trajRunning());

  // 切挡、STOP、遥控掉线都会取消动作
  rc.switch_ = {RC::UP, RC::UP};
  controlCycle();
  rc.switch_ = {RC::MID, RC::UP};
  controlCycle();
  CHECK(arm.trajRunning());
  rc.switch_ = {RC::UP, RC::UP};
  controlCycle();
  CHECK(!arm.trajRunning());
  zeroDerivatives(arm.reference());
  rc.switch_ = {RC::MID, RC::UP};
  controlCycle();
  CHECK(arm.trajRunning());
  rc.switch_.r = RC::DOWN;
  controlLoop();
  CHECK(!arm.trajRunning() && arm.mode() == Arm::Mode::Disabled);
  rc.switch_ = {RC::UP, RC::UP};
  controlCycle();
  rc.switch_ = {RC::MID, RC::UP};
  controlCycle();
  CHECK(arm.trajRunning());
  rc.connect_.online = false;
  controlLoop();
  CHECK(!arm.trajRunning() && arm.mode() == Arm::Mode::Disabled);
  rc.connect_.online = true;

  // 标定挡保持失能并清零
  rc.switch_ = {RC::MID, RC::MID};
  controlCycle();
  CHECK(arm.calibrated() && !arm.trajRunning());
  CHECK(arm.reference().q[0] == 0 && arm.reference().q[1] == 0);

  // 多圈累计角度下的完整动作
  CHECK(arm.calibrate(4 * pi, 6 * pi));
  rc.switch_ = {RC::MID, RC::UP};
  controlCycle();
  CHECK(arm.trajRunning());
  for (int i = 0; i < 2000; ++i) controlCycle();
  const auto multi = arm.reference();
  const auto p = ArmKinematics::forward(multi.q);
  CHECK(near(p.x, .1f) && near(p.y, .1f));
  CHECK(multi.q[0] > 4 * pi - 1.f && multi.q[0] < 4 * pi + 1.f);
  CHECK(multi.q[1] > 6 * pi && multi.q[1] < 6 * pi + 2.f);
  squareEdges(true);
  CHECK(near(multi.q[0] - arm.reference().q[0], 2 * pi, 2e-5f));
  CHECK(near(multi.q[1], arm.reference().q[1], 2e-5f));
  rc.switch_ = {RC::MID, RC::MID};
  controlCycle();
  CHECK(!arm.trajRunning() && arm.mode() == Arm::Mode::Disabled);
}

void startAction() {
  rc.switch_ = {RC::MID, RC::MID};
  controlCycle();
  rc.switch_ = {RC::MID, RC::UP};
  controlCycle();
  CHECK(arm_start_ok && arm_step == 1 && arm.trajRunning());
}

void squareFailureTests() {
  startAction();
  for (int i = 0; i < 2000; ++i) controlCycle();
  squareEdges(true);

  // 预备中取消、切挡、校准、STOP、电机掉线都终止动作且不重试
  for (int reason = 0; reason < 5; ++reason) {
    startAction();
    CHECK(arm.trajRunning());
    if (reason == 0) {
      arm.trajAbort();
    } else if (reason == 1) {
      rc.switch_ = {RC::UP, RC::UP};
    } else if (reason == 2) {
      rc.switch_ = {RC::MID, RC::MID};
    } else if (reason == 3) {
      rc.switch_.r = RC::DOWN;
    } else {
      control_m1.connect_.online = false;
    }
    if (reason == 3 || reason == 4) {
      ++tick;
      controlLoop();
    } else {
      controlCycle();
    }
    CHECK(!arm.trajRunning() && !arm.trajFinished());
    if (reason >= 2) CHECK(arm.mode() == Arm::Mode::Disabled);
    rc.connect_.online = control_m1.connect_.online = true;
    // STOP 和掉线走实际控制主循环，其余保持当前挡位
    for (int i = 0; i < 1000; ++i) {
      if (reason == 3 || reason == 4) {
        ++tick;
        controlLoop();
      } else {
        controlCycle();
      }
    }
    CHECK(arm_step == 0 && !arm.trajRunning());
    zeroDerivatives(arm.reference());
  }

  // 衔接前规划目标变成伸直奇异位：该段被拒绝，后续不再启动
  startAction();
  while (arm_step < 2 || arm.trajRunning()) controlCycle();
  CHECK(arm.trajFinished());
  Arm::Reference singular;
  CHECK(arm.setReference(singular));
  const auto held = arm.reference();
  controlCycle(false);
  CHECK(!arm_start_ok && arm_step == 0 && !arm.trajRunning());
  CHECK(!arm.trajFinished());
  for (int i = 0; i < 1000; ++i) controlCycle();
  CHECK(!arm.trajRunning() && arm_step == 0);
  CHECK(arm.reference().q[0] == held.q[0]);
  zeroDerivatives(arm.reference());

  // 反馈在限位内，但最近一圈的预备目标越限
  rc.switch_ = {RC::MID, RC::MID};
  controlCycle();
  CHECK(arm.calibrate(0, 63.f));
  rc.switch_ = {RC::MID, RC::UP};
  controlCycle();
  CHECK(!arm_start_ok && arm_step == 0 && !arm.trajRunning());
  for (int i = 0; i < 1000; ++i) controlCycle();
  CHECK(!arm.trajRunning() && arm_step == 0);
}
}  // namespace

uint32_t HAL_GetTick() { return tick; }
namespace math {
float rad2deg(const float& q) { return q * (180.f / pi); }
float deg2rad(const float& q) { return q * (pi / 180.f); }
float limit(float value, const float& min, const float& max) {
  return value < min ? min : (value > max ? max : value);
}
}  // namespace math

int main() {
  kinematicsTests();
  trajectoryTests();
  armTests();
  plannedStartTests();
  jointTrajectoryTests();
  controlTests();
  squareFailureTests();
  std::printf("PASS: %d checks (math, Arm, control integration; hardware mocked)\n",
              checks);
}
