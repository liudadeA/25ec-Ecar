/**
 * @file uart_driver.h
 * @brief MSPM0G3507 UART0 驱动 (基于SysConfig配置)
 * @version 1.0
 */

#pragma once

#include <stdint.h>
#include "ti_msp_dl_config.h"

#ifdef __cplusplus
extern "C" {
#endif

// =======================环形缓冲区配置==================
#define RING_BUFFER_SIZE 256
#define MAX_FRAME_SIZE 6

typedef struct {
    uint8_t buffer[RING_BUFFER_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
    volatile uint16_t count;
} RingBuffer;

//=======================读取数据帧配置======================
// 帧解析状态枚举
typedef enum {
    FRAME_WAIT_HEADER,   // 等待包头
    FRAME_IN_DATA,       // 接收数据中
    FRAME_WAIT_FOOTER1,  // 等待包尾1
    FRAME_WAIT_FOOTER2,  // 等待包尾2
    FRAME_WAIT_FOOTER3   // 等待包尾3
} FrameState;

extern RingBuffer uart0_rx_buffer;

void uart_Init();
void uart_send_char(UART_Regs* uart,char ch);
//串口发送字符串
void uart_send_string(UART_Regs* uart ,char* str);
void uart_send_packet(UART_Regs* uart, uint8_t* data, uint16_t length);

void RingBuffer_Init(RingBuffer *rb);
void RingBuffer_Write(RingBuffer *rb, uint8_t data);
uint8_t RingBuffer_Read(RingBuffer *rb);
uint16_t GetPacket(RingBuffer *usartx_rx_buffer, char* out_buf);


#ifdef __cplusplus
}
#endif