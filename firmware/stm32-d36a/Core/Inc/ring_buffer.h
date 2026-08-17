#ifndef __RING_BUFFER_H
#define __RING_BUFFER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <string.h>

// 环形缓冲区配置
#define RING_BUFFER_SIZE 256
#define MAX_FRAME_SIZE 32
#define MAX_BT_FRAME_SIZE 32
#define TEST_STRING "usart1"
#define RESPONSE_STRING "usart1ok\r\n"

typedef struct {
    uint8_t buffer[RING_BUFFER_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
    volatile uint16_t count;
} RingBuffer;

extern RingBuffer usart1_rx_buffer;  // USART1接收环形缓冲??
extern RingBuffer usart2_rx_buffer;  // USART2接收环形缓冲??

void RingBuffer_Init(RingBuffer *rb);
void RingBuffer_Write(RingBuffer *rb, uint8_t data);
uint8_t RingBuffer_Read(RingBuffer *rb);

#ifdef __cplusplus
}   
#endif

#endif /* __RING_BUFFER_H */
