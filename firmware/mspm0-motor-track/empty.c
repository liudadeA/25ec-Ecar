

#include "ti_msp_dl_config.h"
#include "board.h"
#include "bsp_tb6612.h"
#include "Four_gpio.h"
#include "uart_driver.h"
#include "oled_hardware_i2c.h"
#include "stdio.h"
#define  delay_ms(X)                     delay_cycles(CPUCLK_FREQ/1000*X)      


uint32_t casee = 3;
uint8_t go = 0;
volatile uint32_t HAL_GetTick = 0 ;
volatile float left_count = 0;
volatile float right_count = 0;
/***********uart变量*********** */
RingBuffer uart0_rx_buffer = {0};
uint16_t len = 0;
char rx[MAX_FRAME_SIZE];
int take = 0;
volatile uint8_t last_state_main;
unsigned char print_buff[256]={0};
/**********巡线变量*********** */
Sensor8_State state = {0}; //传感器状态
uint8_t Turn_Time = 0; //十字楼口转弯时间计时器
volatile uint8_t Cross_Counter = 0; // 十字路口计数器
volatile uint32_t NO_Cross_time = 0; //十字路口转弯赋值为 HAL_GetTick,4s后清零（计算过一段直线的时间）
/************ 按键全局变量声明***********/
// 中断使用计数
volatile uint8_t my_task = 0;
volatile uint8_t system_go = 0;
volatile uint8_t count_N = 0;
// 中断设置完成任务分配
uint8_t taking_task;
uint8_t taking_N; 



bool casee_change (void)
{
    uint32_t current_time_1 = HAL_GetTick;
    static uint32_t last_control_time_1 = 0;
    static uint32_t interval_1 = 0;
    go = 0;
    interval_1 = current_time_1 - last_control_time_1;
    if (interval_1 >= 10) 
    {   
        //casee ++;
        
        last_control_time_1 = current_time_1;
        go = 1;
        return true;
    }
    return false;
}

int main(void)
{
    SYSCFG_DL_init();
    OLED_Init();
    TB6612_Motor_Stop();
    // 电机中断
    NVIC_EnableIRQ(GPIOB_INT_IRQn); 
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
    // 按键中断
    Motor_PID_Init();
    OLED_ShowString(0,0,(uint8_t *)"preparing...",16);
    while(HAL_GetTick < 2000)
    {
        if(casee_change())
        {
            OLED_ShowString(0,0,(uint8_t *)"preparing...",16);
            OLED_ShowNum(50, 2, HAL_GetTick, 4, 16);
        }
    }
    // delay_ms(2000);  // 系统预热
    // lc_printf("System Start\r\n");
    //等待设置任务 OLED显示
    OLED_Clear();
    uint32_t ms;
    while(system_go < 10)
    {
        if(ms > 8000*2)
        {
            // oled显示
            // 任务赋值
            // 显示固定提示信息（顶部状态栏）
            OLED_ShowString(0, 0, (uint8_t *)"waiting...  ", 16);  // 字体大小改为16避免压缩[3,7](@ref)
            OLED_ShowNum(70, 0, HAL_GetTick, 8, 8);
            // 串口打印启动提示
            lc_printf("wait for starting...\r\n");

            if(count_N>5)count_N=0;

            // 显示任务计数（第一行）
            OLED_ShowString(0, 2, (uint8_t *)"N:", 16);      // Y坐标按行号递增（每行=8像素）
            OLED_ShowNum(100, 2, count_N, 1, 16);             // X坐标右移对齐数字，字体同步为16

            // 显示当前任务（第二行）
            OLED_ShowString(0, 4, (uint8_t *)"Task:", 16);   // 行号+1（Y=2对应第2行）
            OLED_ShowNum(100, 4, my_task, 1, 16);            // 根据标签长度调整X坐标

            // 显示系统状态（第四行，预留第三行可扩展）
            OLED_ShowString(0, 6, (uint8_t *)"care go:", 16); // Y=4跳过第3行[3](@ref)
            OLED_ShowNum(100, 6, system_go, 1, 16);           // 对齐长标签
            ms = 0;

        }
        ms ++;
    }



    taking_N = count_N;
    taking_task = my_task;
    // 确保系统开始不是误触
    //oled显示
    lc_printf("N:  %d  ,task:  %d  \r\n",count_N,my_task);
    lc_printf("System Start\r\n");
    OLED_Clear();
    OLED_ShowString(0,0,(uint8_t *)"Successfully Start",8);
    // 显示任务计数（第一行）
    OLED_ShowString(0, 2, (uint8_t *)"N:", 16);      // Y坐标按行号递增（每行=8像素）
    OLED_ShowNum(50, 2, count_N, 1, 16);             // X坐标右移对齐数字，字体同步为16
    OLED_ShowNum(100, 2, taking_N, 1, 16);             // X坐标右移对齐数字，字体同步为16
    // 显示当前任务（第二行）
    OLED_ShowString(0, 4, (uint8_t *)"Task:", 16);   // 行号+1（Y=2对应第2行）
    OLED_ShowNum(50, 4, my_task, 1, 16);            // 根据标签长度调整X坐标
    OLED_ShowNum(100, 2, taking_task, 1, 16);             // X坐标右移对齐数字，字体同步为16
    // 显示系统状态（第四行，预留第三行可扩展）
    OLED_ShowString(0, 6, (uint8_t *)"care go:", 16); // Y=4跳过第3行[3](@ref)
    OLED_ShowNum(100, 6, system_go, 1, 16);           // 对齐长标签
    // 任务赋值分类
    OLED_Clear();//清屏一下子
    uint8_t Cross_Counter_last = 0   ;

    while (1) 
    {
        // if(Cross_Counter_last != Cross_Counter)
        // {
        //     OLED_ShowString(0, 4, (uint8_t *)"Cross_Counter:", 16);   // 行号+1（Y=2对应第2行）
        // OLED_ShowNum(100, 4, Cross_Counter, 2, 16);
        // Cross_Counter_last = Cross_Counter;
        // }
        
        
        if(casee_change())
        {
            GPIO_Get_8Digital(&state);
            get_outermost_sensor(&state);
            if(NO_Cross_time && (HAL_GetTick - NO_Cross_time) < 2000) //十字路口转弯结束后的2s内强制不进入十字路口状态
            {
                if(state.state == 10)
                {
                    state.state = 4;
                }
            }
            Eight_LineTrack(state.state);
        }
    }
}


//实现了中断读取编码器示数的功能，左轮右轮前进为正数，后退为负数，一圈大概10234，除以10234，再乘100，实现每转一圈为100
void GROUP1_IRQHandler(void)
{
    static uint8_t debunce_time = 0;       // 消抖延时

    uint32_t gpioB = DL_GPIO_getEnabledInterruptStatus(GPIOB,DL_GPIO_PIN_13 | DL_GPIO_PIN_15
     | Task_Set_Start_Set_PIN |Task_Set_COUNT_PIN | Task_Set_TASK_SET_PIN | Task_Set_Count_Zero_PIN);
    if((gpioB & DL_GPIO_PIN_13) == DL_GPIO_PIN_13)
    {
        if(DL_GPIO_readPins(GPIOB,DL_GPIO_PIN_14))
        {
            left_count ++;
        }
        else
        {
            left_count -- ;
        }
        DL_GPIO_clearInterruptStatus(GPIOB,DL_GPIO_PIN_13);
    }
    if((gpioB & DL_GPIO_PIN_15) == DL_GPIO_PIN_15 )
    {
        if(DL_GPIO_readPins(GPIOB,DL_GPIO_PIN_16))
        {
            right_count --;
        }
        else
        {
            right_count ++;
        }
      
        DL_GPIO_clearInterruptStatus(GPIOB,DL_GPIO_PIN_15);
    }
    // 按键中断
    if((gpioB & Task_Set_Start_Set_PIN) == Task_Set_Start_Set_PIN )
    {
        debunce_time = HAL_GetTick;
        while(HAL_GetTick - debunce_time < 500) {};
        if (!DL_GPIO_readPins(GPIOB,Task_Set_Start_Set_PIN)) 
        {
            system_go ++;
            // oled显示
        }
        DL_GPIO_clearInterruptStatus(GPIOB,Task_Set_Start_Set_PIN);
    }
    if((gpioB & Task_Set_COUNT_PIN) == Task_Set_COUNT_PIN )
    {
        debunce_time = HAL_GetTick;
        while(HAL_GetTick - debunce_time < 300) {};
        if (!DL_GPIO_readPins(GPIOB,Task_Set_COUNT_PIN)) 
        {
            count_N ++;
            // oled显示
            lc_printf("N = : %d\r\n",count_N);

        }
        DL_GPIO_clearInterruptStatus(GPIOB,Task_Set_COUNT_PIN);
    }
    if((gpioB & Task_Set_TASK_SET_PIN) == Task_Set_TASK_SET_PIN )
    {
        debunce_time = HAL_GetTick;
        while(HAL_GetTick - debunce_time < 300) {};
        if(!DL_GPIO_readPins(GPIOB,Task_Set_TASK_SET_PIN))
        {
            my_task = (my_task + 1) % 2;
        }
        DL_GPIO_clearInterruptStatus(GPIOB,Task_Set_TASK_SET_PIN);
    }
    if((gpioB & Task_Set_Count_Zero_PIN) == Task_Set_Count_Zero_PIN )
    {
        debunce_time = HAL_GetTick;
        while(HAL_GetTick - debunce_time < 300) {};
        if(!DL_GPIO_readPins(GPIOB,Task_Set_Count_Zero_PIN))
        {
            count_N = (count_N + 1)%2;
            lc_printf("N = : %d\r\n",count_N);
        }
        DL_GPIO_clearInterruptStatus(GPIOB,Task_Set_Count_Zero_PIN);
    }

}
//主要是实时更新电机的速度和圈数,//每10ms。
void TIMA0_IRQHandler(void)
{
    switch (DL_TimerA_getPendingInterrupt(TIMER_0_INST)) {
        case DL_TIMER_IIDX_ZERO:
        left_motor.current_quanshu = left_count/8910*100;
        right_motor.current_quanshu = right_count/8910*100;
        // 计算转速（RPM），单位是转/分钟
        left_motor.current_speed = (left_motor.current_quanshu-left_motor.last_quanshu)*60.0f*1000.0f/10.0f/100.0f;   //  每分钟多少转
        right_motor.current_speed =(right_motor.current_quanshu-right_motor.last_quanshu)*60.0f*1000.0f/10.0f/100.0f;        //  每分钟多少转
            // PID_Compute();可以选择单个，或者是内外环
            // Motor_Control();这里可以用状态机去执行操作 (直行就正常操作即可，拐弯或者掉头就用一个while循环限制进去，直到完成再跳出循环) 
            // ????也可以放在while内进行，因为PID已经计算完了
        left_motor.last_quanshu = left_motor.current_quanshu ;
        right_motor.last_quanshu = right_motor.current_quanshu;
        left_motor.last_speed = left_motor.current_speed ;
        right_motor.last_speed = right_motor.current_speed ;
        // lc_printf("%.2f,%.2f\r\n",left_motor.target_speed,left_motor.current_speed);
        // lc_printf("%.2f,%.2f\r\n",left_motor.current_quanshu,right_motor.current_quanshu);
        //lc_printf("%.2f,%.2f\r\n",right_motor.target_quanshu,right_motor.current_quanshu);




            break;
        default:
            break;
    }

}

void SysTick_Handler(void)
{
    HAL_GetTick ++;
}