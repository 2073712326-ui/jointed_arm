#pragma once
#include <cstdint>
uint32_t HAL_GetTick();
struct TestUart {
  struct {
    int BaudRate{}, WordLength{}, StopBits{}, Parity{}, Mode{}, HwFlowCtl{}, OverSampling{};
  } Init;
};
extern TestUart test_uart;
#define RC_UART (&test_uart)
enum { UART_WORDLENGTH_9B, UART_STOPBITS_1, UART_STOPBITS_2, UART_PARITY_EVEN,
       UART_MODE_RX, UART_HWCONTROL_NONE, UART_OVERSAMPLING_16, HAL_OK };
inline int HAL_UART_Init(TestUart*) { return HAL_OK; }
inline void Error_Handler() {}
