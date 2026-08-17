#include "ring_buffer.h"
#include "usart.h"


// 初始化环形缓冲区
void RingBuffer_Init(RingBuffer *rb) {
    rb->head = 0;
    rb->tail = 0;
    rb->count = 0;
    memset(rb->buffer, 0, RING_BUFFER_SIZE);
}

// 向环形缓冲区写入数据
void RingBuffer_Write(RingBuffer *rb, uint8_t data) {
    if (rb->count < RING_BUFFER_SIZE) {
        rb->buffer[rb->head] = data;
        rb->head = (rb->head + 1) % RING_BUFFER_SIZE;
        rb->count++;
    }
}

// 从环形缓冲区读取数据
uint8_t RingBuffer_Read(RingBuffer *rb) {
    uint8_t data = 0;
    if (rb->count > 0) {
        data = rb->buffer[rb->tail];
        rb->tail = (rb->tail + 1) % RING_BUFFER_SIZE;
        rb->count--;
    }
    return data;
}
