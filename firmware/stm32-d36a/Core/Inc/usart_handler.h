#ifndef __USART_HANDLER_H
#define __USART_HANDLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ring_buffer.h"
#include "main.h"


// 帧解析状态枚举
typedef enum {
    FRAME_WAIT_HEADER,   // 等待包头
    FRAME_IN_DATA,       // 接收数据中
    FRAME_WAIT_FOOTER1,  // 等待包尾1
    FRAME_WAIT_FOOTER2,  // 等待包尾2
    FRAME_WAIT_FOOTER3   // 等待包尾3
} FrameState;

extern FrameState frame_state;
extern uint16_t frame_index;
extern uint8_t frame_buffer[MAX_FRAME_SIZE];

// USART1处理接收的数据
void USART1_ProcessReceivedData(void);

// USART2处理接收到的数据
void USART2_ProcessReceivedData(void);

//每个USART都可以直接调用
uint16_t GetPacket(UART_HandleTypeDef *huart, RingBuffer *usartx_rx_buffer, char* out_buf);


// 通用数据包发送
void Send_Packet(UART_HandleTypeDef *huart,const char* data,uint16_t len);
// 向串口3发送数据， 包头包尾在数组中写
void Send_Packet_To_Motor(uint8_t* data,uint16_t len);

// 处理视觉传回数据
void Process_VisionData(char *packet,uint16_t axis[2]);

#ifdef __cplusplus
}
#endif

#endif /* __USART_HANDLER_H */
