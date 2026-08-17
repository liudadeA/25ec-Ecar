/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//全局变量定义
#include "stdio.h"
#include "stdlib.h"
#include "usart_handler.h"
#include "ring_buffer.h"
#include <string.h>

// 添加定时器句柄的外部声明
extern TIM_HandleTypeDef htim8;


volatile uint32_t step_count_A = 0;
volatile uint32_t step_target_A = 0;
volatile uint32_t arr_value_A = 999;
volatile uint8_t motorA_running = 0;

volatile uint32_t step_count_B = 0;
volatile uint32_t step_target_B = 0;
volatile uint32_t arr_value_B = 999;
volatile uint8_t motorB_running = 0;

volatile  int32_t steps_x  = 0 ;
volatile    int32_t steps_y  = 0 ;
uint32_t casee = 0; 

volatile uint16_t kaiguan = 0;



// 添加快速转动相关的全局变量
volatile uint8_t motorA_fast_rotation = 0;  // 快速转动标志  0-2是一直循环60°转，在串口中断中设置为3
volatile uint32_t fast_rotation_timer = 0;   // 快速转动计时器

// UART环形缓冲区【通信】
RingBuffer usart1_rx_buffer = {0};
RingBuffer usart2_rx_buffer = {0};
RingBuffer usart3_rx_buffer = {0};

char rx_frame[MAX_FRAME_SIZE]; //从环形缓冲器读出第一个完整帧
uint16_t len = 0;
uint8_t rx = 0; //是否读帧
extern FrameState frame_state;
extern uint16_t frame_index;
extern uint8_t frame_buffer[MAX_FRAME_SIZE];
// 处理接收的视觉数据
uint16_t axis[2] = {0};
//初始化通信【通信】
void System_Init(void)
{
    // 初始化环形缓冲区
    RingBuffer_Init(&usart1_rx_buffer);
    RingBuffer_Init(&usart2_rx_buffer);
    
    // 启动串口中断接收
    HAL_UART_Receive_IT(&huart1, &usart1_rx_buffer.buffer[usart1_rx_buffer.head], 1);
    HAL_UART_Receive_IT(&huart2, &usart2_rx_buffer.buffer[usart2_rx_buffer.head], 1);
}



//第一批简单代码
//使能/失能
void MotorA_Enable(void)  { HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_SET); }
void MotorA_Disable(void) { HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET); }
void MotorB_Enable(void)  { HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET); }
void MotorB_Disable(void) { HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET); }
//方向控制         1=正转，0=反转
void MotorA_SetDir(uint8_t dir) { HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, dir ? GPIO_PIN_SET : GPIO_PIN_RESET); }
void MotorB_SetDir(uint8_t dir) { HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, dir ? GPIO_PIN_SET : GPIO_PIN_RESET); }


// freq单位：Hz     freq的范围是152.6HzD——500kHZ
void MotorA_SetSpeed(uint32_t freq)
{
	uint32_t tim8_clock = 84000000; // 
	uint32_t psc = 83;              // 预分频器            1MHz定时器时钟
	uint32_t actual_clock = tim8_clock / (psc + 1); // 实际定时器时钟1MHz
    arr_value_A = (uint32_t)(actual_clock / freq) - 1; //    

}

void MotorB_SetSpeed(uint32_t freq)
{   uint32_t tim8_clock = 84000000; // TIM8实际时钟84MHz
	uint32_t psc = 83;              // 预分频器
	uint32_t actual_clock = tim8_clock / (psc + 1); // 实际定时器时钟1MHz
    arr_value_B = (uint32_t)(actual_clock/ freq) - 1;

}


//第二批代码
//X轴转动函数，正数正转，负数反转。6400/（1.8/32） = 360°
void MotorA_Run(int32_t steps)
{    
	if(steps != 0){
		step_count_A = 0;
		if(steps > 0)
		{
		MotorA_SetDir(1);
		step_target_A = steps;
		}
		if(steps < 0)
		{
		MotorA_SetDir(0);
		step_target_A = abs(steps)   ;
	
		}
		__HAL_TIM_SET_AUTORELOAD(&htim8, arr_value_A);
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, arr_value_A/2);
		motorA_running = 1;
//   	 MotorA_Enable();
		HAL_TIM_PWM_Start_IT(&htim8, TIM_CHANNEL_1);
		HAL_TIM_Base_Start_IT(&htim8);
}
	else
	{
		HAL_TIM_PWM_Stop_IT(&htim8, TIM_CHANNEL_1); //关闭TIM8的ch1通道的中断
//		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET) ;		
	}
}
//Y轴转动函数，正数正转，负数反转。6400/（1.8/32） = 360°
void MotorB_Run(int32_t steps)
{
	if(steps != 0)
	{
		step_count_B = 0;
		if(steps > 0)
		{
		MotorB_SetDir(1);
		step_target_B = steps;
		}
		if(steps < 0)
		{
		MotorB_SetDir(0);
		step_target_B = abs(steps)   ;
	
		}
		__HAL_TIM_SET_AUTORELOAD(&htim8, arr_value_B);
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, arr_value_B/2);
		motorB_running = 1;
//   	 MotorB_Enable();
		HAL_StatusTypeDef status = HAL_TIM_PWM_Start_IT(&htim8, TIM_CHANNEL_2);
		HAL_TIM_Base_Start_IT(&htim8);
}
	else
	{
		HAL_TIM_PWM_Stop_IT(&htim8, TIM_CHANNEL_2);            //关闭TIM8的ch2通道的中断		
	}
}	



//X轴一直转动函数，即没识别到对象一直正方向转动的函数
void Run_continue(void)
{
	MotorA_SetDir(1);
	MotorA_SetSpeed(4000);
	__HAL_TIM_SET_AUTORELOAD(&htim8, arr_value_A);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, arr_value_A/2);
	HAL_TIM_PWM_Start_IT(&htim8, TIM_CHANNEL_1);
	HAL_TIM_Base_Start_IT(&htim8);
	
}

// 快速转动功能 - 转60度停300ms
void MotorA_FastRotate60Degrees(void)
{
    if(motorA_fast_rotation == 0) {
        // 开始快速转动
        int32_t steps = 4000;  // 60度对应的步数 (60/1.8*32)
        MotorA_SetSpeed(1000);  // 设置更高速度
        MotorA_Run(steps);
        motorA_fast_rotation = 1;
        fast_rotation_timer = HAL_GetTick();
    }

    // 处理电机A的快速转动
    if(motorA_fast_rotation == 1 && !motorA_running) {
        // 电机停止，开始等待300ms
        motorA_fast_rotation = 2;
        fast_rotation_timer = HAL_GetTick();
    }
    else if(motorA_fast_rotation == 2) {
        // 检查是否等待了300ms
        if(HAL_GetTick() - fast_rotation_timer >= 600) {
            motorA_fast_rotation = 0;  // 重置状态
               // 恢复正常速度
        } 
		MotorA_SetSpeed(500);   
    }
}
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//这是usart1重定向，用和电脑传输数据的【通信】
#if 1
#pragma import(__use_no_semihosting)             
#endif
struct __FILE 
{ 
	int handle; 
	/* Whatever you require here. If the only file you are using is */ 
	/* standard output using printf() for debugging, no file handling */ 
	/* is required. */ 
}; 
/* FILE is typedef’ d in stdio.h. */ 
FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 

//重定义fputc函数 
int fputc(int ch, FILE *f)
{
	while((USART1->SR&0X40)==0);//printf 定向到usartx
	USART1->DR = (uint8_t) ch;      	
	return ch;
}
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */



//PID相关内容
//1. PID结构体定义
typedef struct {
    float kp;        // 比例系数
    float ki;        // 积分系数  
    float kd;        // 微分系数
    float error;     // 当前误差
    float last_error; // 上次误差
    float integral;   // 积分项
    float output;     // 输出值
    float max_output; // 输出限幅
    float integral_limit; // 积分限幅
} PID_Controller;

// 为X轴和Y轴分别创建PID控制器
PID_Controller pid_x = {0};
PID_Controller pid_y = {0};

//2. PID初始化函数
 void PID_Init(PID_Controller *pid, float kp, float ki, float kd, float max_output, float integral_limit)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->error = 0;
    pid->last_error = 0;
    pid->integral = 0;
    pid->output = 0;
    pid->max_output = max_output;
    pid->integral_limit = integral_limit;
}

// 初始化X轴和Y轴的PID
void AimingPID_Init(void)
{
    // X轴PID参数（水平方向）
    PID_Init(&pid_x, 0.05,0.0, 0.04, 10, 50);
												
    // Y轴PID参数（垂直方向）
    PID_Init(&pid_y, 0.5, 0.0, 0.01, 10, 50);
}

//3. PID计算函数
float PID_Calculate(PID_Controller *pid, float setpoint, float feedback)
{
    // 计算误差
    pid->error = setpoint - feedback;     
    
    // 比例项
    float p_term = pid->kp * pid->error;
    
    // 积分项
    pid->integral += pid->error;
    
    // 积分限幅
    if(pid->integral > pid->integral_limit) {
        pid->integral = pid->integral_limit;
    } else if(pid->integral < -pid->integral_limit) {
        pid->integral = -pid->integral_limit;
    }
    
    float i_term = pid->ki * pid->integral;
    
    // 微分项
    float d_term = pid->kd * (pid->error - pid->last_error);
    
    // 计算输出
    pid->output = p_term + i_term + d_term;
    
    // 输出限幅
    if(pid->output > pid->max_output) {
        pid->output = pid->max_output;
    } else if(pid->output < -pid->max_output) {
        pid->output = -pid->max_output;
    }
    
    // 更新上次误差
    pid->last_error = pid->error;
    
    return pid->output;
}

//4. 瞄准控制主函数
void LaserAimingPID_Control(float target_x, float target_y, float current_x, float current_y)
{
    // 计算X轴控制输出
    float output_x = PID_Calculate(&pid_x, target_x, current_x);
    
    // 计算Y轴控制输出
    float output_y = PID_Calculate(&pid_y, target_y, current_y);
    
    // 转换为步数
     steps_x = (int32_t)output_x;
     steps_y = (int32_t)output_y;
    
    // 死区控制，避免微小误差导致的抖动
    if(abs(steps_x) > 5) {
        MotorA_Run(steps_x);  // X轴电机
		
    }
    else{
		MotorA_Run(0)   
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET) ;	
		
	}
    if(abs(steps_y) > 32) {
        MotorB_Run(steps_y);  // Y轴电机
		
    }
	else{
		  MotorB_Run(0)     ;
	}
    
    // 打印调试信息
//    printf("目标: (%.2f, %.2f), 当前: (%.2f, %.2f)\r\n", 
//           target_x, target_y, current_x, current_y);
//    printf("输出: X=%d, Y=%d\r\n", steps_x, steps_y);
}



/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**************************************************************************
函数功能：校验位计算
入口参数：需要计算校验位的数组
返回  值：无
**************************************************************************/
uint8_t Check_Sum(uint8_t *array)
{
	uint8_t result=0;
  int k=0;
	for(k=0;k<9;k++)
	{
	result=result^array[k];
	}
	return result;	
}

  void Motor_Y(uint8_t speed,int8_t y_position)
  {
	                  //帧头   地址 模式  转向 细分    位置      速度    校验  帧尾巴
	  uint8_t data_1[11]={0x7b,0x01,0x02,0x01,0x20,0x00,0x00,0x00,0xC8,0x00,0x7d}     ;
	  data_1[8] = speed;
	  if(y_position > 0)
	  {
		  data_1[6] =  y_position    ;
	  }
	 else if (y_position <= 0)
	 {                             
		   data_1[3] = 0;
		   data_1[6] = abs( y_position);  
	 }
	  data_1[9] = Check_Sum(data_1)  ;
	  Send_Packet_To_Motor(data_1,sizeof(data_1));
	 HAL_Delay(10);
  }
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	
	//PID初始化函数
	AimingPID_Init ();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM8_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

    // 添加这些调试信息



/* 以下是测试例子代码


	    // 电机A正转，电机B反转
//    MotorA_Enable();
//    MotorA_SetDir(1); // 1=正转，0=反转
//    MotorA_SetSpeed(1000); // 1000Hz

    MotorB_Enable();
//    MotorB_SetDir(1);
//    MotorB_SetSpeed(1000); // 500Hz
*/       	
	System_Init();  //初始化通信【通信】
	
	
	
	//等待电源上电稳定
	HAL_Delay(1000);
    //使能  	
	uint8_t data[11] = {0x7b,0x01,0x04,0x01,0x20,0x00,0x00,0x00,0xC8,0x97,0x7d};
//  uint8_t data[11] = {0x7b,0x01,0x01,0x01,0x20,0x00,0x00,0x00,0x64,0x3E,0x7d} ;   //一直转
	Send_Packet_To_Motor(data,sizeof(data));
	MotorA_Enable();
//    MotorB_Run(0);
    MotorA_Run(0);
	//等待使能完毕	
	HAL_Delay(500);
	//设置步进电机的速度（利用频率来控制）
//	MotorA_SetSpeed(3000);   // 静态200到500比较合适

	MotorA_SetSpeed(300);   // 00Hz
//	MotorB_SetSpeed(0);
//	Run_continue();
//MotorB_Run(1800);
//MotorA_Run(0);
//	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET) ;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */



  
  
  //获得开关的0/1值，开关关上是0吧
 kaiguan = HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_15);
  
  while (1)
  {
	
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
/*	之前测试的程序  
        // 可以根据motorA_running/motorB_running判断是否运动完成
//    static uint32_t last_counter = 0;
//    uint32_t current_counter = __HAL_TIM_GET_COUNTER(&htim8);
//    
//    if(current_counter != last_counter)
//    {
//        printf("TIM8计数器: %d\r\n", current_counter);
//        last_counter = current_counter;
//    }
//    
//    if(!motorB_running)
//    {
//        printf("所有电机已停止\r\n");
//    }
//    
//    if(!motorA_running)
//		MotorA_Run(0);
//	else 
//	{
//		printf("DELAY")      ;
//		HAL_Delay(3000);
//	}
*/ 
 /*状态机测试程序  
  switch(casee)
	{
		case 0 :
			 MotorA_Run(0);  //停止
			 MotorB_Run(0);  //停止

		
		
		break;
		case 1 :
			  Run_continue();  //一直转
			  printf("一直转")  ;
			  MotorA_Run(-10);        //逆时针
		
		break;
		case 2 :
			MotorA_SetSpeed(500);   // 00Hz
			MotorB_SetSpeed(500);
		    MotorA_Run(10);    //顺时针
			MotorA_Run(600);
		
		break;
	}
		MotorA_Run(-10);
*/



	   //X轴进行中心坐标寻找
		if(kaiguan==1){
	   if(motorA_fast_rotation != 3)
	   {
		   MotorA_FastRotate60Degrees();
	   }
	   if(motorA_fast_rotation == 3)
	   {
		   LaserAimingPID_Control(320,240,axis[0],240);  
	   }
   }
		if(kaiguan!= 1){
		//画圆操作
		
		}

  
	   printf("%d,%d,%d\r\n",200,axis[0],steps_x) ;




	
	
	// 处理通信测试
	if(1){
		len = GetPacket(&huart2, &usart2_rx_buffer, rx_frame);
		Process_VisionData(rx_frame,axis);
		HAL_Delay(50);
	}
  }
 
  
 
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
//第二批简单代码
//中断回调函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
//	printf("j进入中断！");
    // 电机A
    if(htim->Instance == TIM8  && motorA_running)
    {
        step_count_A++;
//		printf("%d\r\n",step_count_A)    ;

//        if(step_count_A < 100 && arr_value_A > 499) arr_value_A -= 5;
//        else if(step_count_A > step_target_A - 100 && arr_value_A < 999) arr_value_A += 5;
//        __HAL_TIM_SET_AUTORELOAD(htim, arr_value_A);
//        __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, arr_value_A/2);

        if(step_count_A >= step_target_A)	
        {
//			printf("停止");
			HAL_TIM_PWM_Stop_IT(htim, TIM_CHANNEL_1); //关闭TIM8的ch1通道的中断
//			HAL_TIM_Base_Stop_IT(htim);  // 直接把定时器TIM8停止定时器关闭
 ;
//			MotorA_Disable();

//			printf("已经停止");
			motorA_running = 0;

        }
    }
    // 电机B
    if(htim->Instance == TIM8 )
    {

		if(motorB_running)
        {
        step_count_B++;
//		printf("%d\r\n",step_count_B)    ;

//        if(step_count_B < 100 && arr_value_B > 499) arr_value_B -= 5;
//        else if(step_count_B > step_target_B - 100 && arr_value_B < 999) arr_value_B += 5;
//        __HAL_TIM_SET_AUTORELOAD(htim, arr_value_B);
//        __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, arr_value_B/2);

        if(step_count_B >= step_target_B)
        {
//			printf("停止");
			HAL_TIM_PWM_Stop_IT(htim, TIM_CHANNEL_2);            //关闭TIM8的ch2通道的中断
//			HAL_TIM_Base_Stop_IT(htim);  //   直接把定时器TIM8停止定时器关闭
//			MotorB_Disable();
//			printf("已经停止");
			motorB_running = 0;

        }
	}
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
