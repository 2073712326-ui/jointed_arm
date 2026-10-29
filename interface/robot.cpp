/**
 ******************************************************************************
 * @file    robot.cpp/h
 * @brief   Main program. 主程序
 * @author  Spoon Guan
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#include "interface/robot.h"

#include "cmsis_os.h"
#include "hardware_config.h"

#include "app/arm.h"
#include "app/can_monitor.h"
#include "app/control.h"
// #include "app/imu_monitor.h"
#include "app/motor_monitor.h"
#include "base/common/math.h"
#include "base/remote/remote.h"
#include "base/remote/wfly.h"

#ifdef RC_UART
#ifdef RF206S
WFLY rc(RC_UART);
#endif
#ifdef DR16
RC rc(RC_UART);
#endif
#else
RC rc;
#endif  // RC_UART

// 累计关节角范围为 ±3600°，单位 rad。
const Arm::Limits arm_limits = {{math::deg2rad(-3600.f), math::deg2rad(-3600.f)}, {math::deg2rad(3600.f),math::deg2rad(3600.f)}};
// 加速度前馈系数 加速度-》力矩没建模
const Arm::Feedforward arm_feedforward = {{0.0f, 0.0f}};
Arm arm(J0, J1, arm_limits, arm_feedforward);

/* FreeRTOS tasks-----------------------------------------------------------*/
osThreadId controlTaskHandle;
//算出应该发送的电机输出
void controlTask(void const* argument) {
  allMotorsInit();//配置电机ID、重置控制器、设置反馈来源
  controlInit();//配置遥控串口和应用初始状态
  rc.init();//启动遥控数据接受
  uint32_t tick = osKernelSysTick();
  for (;;) {
    rc.handle();//把最近接收到的数据解码为摇杆和拨杆状态
    controlLoop();//处理机器人模式，准备机械臂目标
    allMotorsHandle();//根据目标和反馈计算电机输出
    osDelayUntil(&tick, 1);//等待下一个周期，目标为周期1ms
  }
}


osThreadId canTaskHandle;
//把最新输出打包发送
void canTask(void const* argument) {
  canFilterInit();
  for (;;) {
    canTxMonitor();
  }
}

// osThreadId imuTaskHandle;
// void imuTask(void const* argument) {
//   imu::initAll();
//   for (;;) {
//     imu::handleAll();
//     osDelay(1);
//   }
// }

// Create and config tasks
void rtosTaskInit(void) {
  osThreadDef(control_task, controlTask, osPriorityAboveNormal, 0, 256);
  controlTaskHandle = osThreadCreate(osThread(control_task), NULL);

  osThreadDef(can_task, canTask, osPriorityHigh, 0, 128);
  canTaskHandle = osThreadCreate(osThread(can_task), NULL);

//   osThreadDef(imu_task, imuTask, osPriorityRealtime, 0, 128);
//   imuTaskHandle = osThreadCreate(osThread(imu_task), NULL);
}
