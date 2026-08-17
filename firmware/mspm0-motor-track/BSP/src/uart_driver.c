/**
 * @file uart_driver.c
 * @brief MSPM0G3507 UART0 驱动实现 (基于SysConfig配置)
 */
#include "ti_msp_dl_config.h"
#include <string.h>
#include "stdio.h"
#include "uart_driver.h"

// 定义包头包尾
const uint8_t g_header[] = {0xAA};
const uint8_t g_footer[] = {0x55, 0x0D, 0x0A}; // \x55\r\n
uint8_t ack[] = {'A','C','K'};
//数据帧缓存，不同串口不需要分开定义
static FrameState frame_state = FRAME_WAIT_HEADER;
static uint16_t frame_index = 0;
static uint8_t frame_buffer[MAX_FRAME_SIZE];

//======================串口初始化=======================//
void uart_Init(){
    // #1. UART0 配置
    //清除串口中断标志
    NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
    //使能串口中断
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);

    // #2. 其他串口使用到再配置
}
//====================串口发送==========================
/*发送不冲突，已经有等待*/
void uart_send_char(UART_Regs* uart,char ch)
{
    //当串口0忙的时候等待，不忙的时候再发送传进来的字符
    while( DL_UART_isBusy(uart) == true );
    //发送单个字符
    DL_UART_Main_transmitData(uart, ch);
}
//串口发送字符串或uint8_t数组
void uart_send_string(UART_Regs* uart ,char* str)
{
    //当前字符串地址不在结尾 并且 字符串首地址不为空
    while(*str!=0&&str!=0)
    {
        //发送字符串首地址中的字符，并且在发送完成之后首地址自增
        uart_send_char(uart,*str++);
    }
}

/**
 * @brief 发送完整数据包（包头 + 数据体 + 包尾）
 * @param uart UART实例指针
 * @param data 待发送数据数组
 * @param length 数据长度
 */
void uart_send_packet(UART_Regs* uart, uint8_t* data, uint16_t length)
{
    // 1. 发送包头（0xAA）
    uart_send_char(uart, g_header[0]);
    
    // 2. 发送数据体（二进制安全）
    for (uint16_t i = 0; i < length; i++) {
        uart_send_char(uart, data[i]);
    }
    
    // 3. 发送包尾（0x55, 0x0D, 0x0A）
    for (uint8_t i = 0; i < sizeof(g_footer); i++) {
        uart_send_char(uart, g_footer[i]);
    }
}

//==================串口中断=========================
//串口的中断服务函数
volatile unsigned char uart0_data = 0;
void UART_0_INST_IRQHandler(void)
{
    //如果产生了串口中断
    switch( DL_UART_getPendingInterrupt(UART_0_INST) )
    {
        case DL_UART_IIDX_RX://如果是接收中断
            //#0. 检查硬件错误
            //#1. 将发送过来的数据保存在变量中
            uart0_data = DL_UART_Main_receiveData(UART_0_INST);
            // #2. 存入环形缓冲区
            RingBuffer_Write(&uart0_rx_buffer, uart0_data);
            // #3.测试，通过串口发送
            uart_send_char(UART_0_INST,uart0_data);
            break;

        default://其他的串口中断
            break;
    }
}

//======================环形缓冲区定义******************** */
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

/********************数据帧读取*********************** */

/***************************************************************************
@brief:接受模版：统一使用的获取数据包到输出缓冲区out_buf
@attention: 
1.提前定义静态变量，不然响应速度太慢
2.只能使用一个串口，使用其他串口需要重写一遍这个函数
***************************************************************************/
uint16_t GetPacket(RingBuffer *usartx_rx_buffer, char* out_buf) 
{
    uint16_t packet_len = 0;
    uart_send_string(UART_0_INST ,"start read frame\r\n");
    while (usartx_rx_buffer->count > 0) {
        uint8_t ch = RingBuffer_Read(usartx_rx_buffer);
        //uart_send_string(UART_0_INST ,"reading frame\r\n");
        switch (frame_state) {
            case FRAME_WAIT_HEADER:
                if (ch == *g_header) {
                    frame_state = FRAME_IN_DATA;
                    frame_index = 0;
                    frame_buffer[frame_index++] = ch;  // 存储帧头
                    //uart_send_string(UART_0_INST ,"Head OK\r\n");
                }
                break;
                
            case FRAME_IN_DATA:
                // 检查缓冲区溢出
                if (frame_index >= MAX_FRAME_SIZE - 1) {
                    frame_state = FRAME_WAIT_HEADER;
                    frame_index = 0;
                    //uart_send_string(UART_0_INST ,"OUT frame\r\n");
                    break;
                }
                
                frame_buffer[frame_index++] = ch;
                
                // 检查是否开始包尾
                if (ch == g_footer[0]) {
                    frame_state = FRAME_WAIT_FOOTER1;
                    //uart_send_string(UART_0_INST ,"FOOTER 0 OK\r\n");
                }
                break;
                
            case FRAME_WAIT_FOOTER1:
                if (frame_index >= MAX_FRAME_SIZE - 1) {
                    frame_state = FRAME_WAIT_HEADER;
                    frame_index = 0;
                    //uart_send_string(UART_0_INST ,"OUT frame\r\n");
                    break;
                }
                
                frame_buffer[frame_index++] = ch;
                
                if (ch == g_footer[1]) {
                    frame_state = FRAME_WAIT_FOOTER2;
                    //uart_send_string(UART_0_INST ,"FOOTER 1 OK\r\n");
                } else {
                    frame_state = FRAME_IN_DATA;
                }
                break;
                
            case FRAME_WAIT_FOOTER2:
                if (frame_index >= MAX_FRAME_SIZE - 1) {
                    frame_state = FRAME_WAIT_HEADER;
                    frame_index = 0;
                    //uart_send_string(UART_0_INST ,"OUT frame\r\n");
                    break;
                }
                
                frame_buffer[frame_index++] = ch;
                
                if (ch == g_footer[2]) {
                    frame_state = FRAME_WAIT_HEADER;  // 直接重置状态机
                    
                    // 复制完整帧到输出缓冲区
                    memcpy(out_buf, frame_buffer, frame_index);
                    packet_len = frame_index;
                    
                    // 添加字符串终止符并打印到USART1
                    out_buf[frame_index] = '\0';
                    
                    //打印到UART0中
                    uart_send_string(UART_0_INST ,"successfully read frame\r\n");
                    uart_send_string(UART_0_INST ,out_buf);
                    uart_send_packet(UART_0_INST, ack, 3); // 测试发包函数
                    // 重置索引并返回包长度
                    frame_index = 0;
                    return packet_len;
                } else {
                    frame_state = FRAME_IN_DATA;
                }
                break;
                
            case FRAME_WAIT_FOOTER3:
                // 该状态已被简化处理
                frame_state = FRAME_WAIT_HEADER;
                frame_index = 0;
                break;
        }
    }
    
    uart_send_string(UART_0_INST ,"faultly read frame\r\n");
    return 0;  // 无完整数据包
}