
#ifndef _BSP_TB6612_H
#define _BSP_TB6612_H

#include "board.h"

#define AIN1_OUT(X)  ( (X) ? (DL_GPIO_setPins(TB6612_PORT,TB6612_AIN1_PIN)) : (DL_GPIO_clearPins(TB6612_PORT,TB6612_AIN1_PIN)) )
#define AIN2_OUT(X)  ( (X) ? (DL_GPIO_setPins(TB6612_PORT,TB6612_AIN2_PIN)) : (DL_GPIO_clearPins(TB6612_PORT,TB6612_AIN2_PIN)) )

#define BIN1_OUT(X)  ( (X) ? (DL_GPIO_setPins(TB6612_PORT,TB6612_BIN1_PIN)) : (DL_GPIO_clearPins(TB6612_PORT,TB6612_BIN1_PIN)) )
#define BIN2_OUT(X)  ( (X) ? (DL_GPIO_setPins(TB6612_PORT,TB6612_BIN2_PIN)) : (DL_GPIO_clearPins(TB6612_PORT,TB6612_BIN2_PIN)) )


void TB6612_Motor_Stop(void);
extern volatile uint32_t HAL_GetTick ;
extern volatile float left_count ;
extern volatile float right_count ;
extern float interval_in;   //隔interval长时间（ms）执行一次PID计算和编码器测数；
extern float interval_out;
extern uint8_t pid_control_enabled;
extern float total_revolutions_right; 
extern float total_revolutions_left; 
// 控制模式枚举
typedef enum {
    CONTROL_MODE_SPEED = 0,
    CONTROL_MODE_POSITION
} ControlMode_t;

//以下是自己写的
// PID控制器结构体
typedef struct {
    float Kp;              // 比例系数
    float Ki;              // 积分系数
    float Kd;              // 微分系数
    
    float setpoint;        // 目标值(RPM)
    float input;           // 当前输入值（反馈RPM）
    float output;          // PID输出值
    
    float error;           // 当前误差
    float last_error;      // 上次误差
    float integral;        // 积分累积
    float derivative;      // 微分值
    
    float output_min;      // 输出最小值
    float output_max;      // 输出最大值
    
    uint32_t last_time;    // 上次计算时间
    uint8_t enabled;       // 是否启用
    
    // 抗积分饱和
    float integral_limit;  // 积分限制值
    float dead_zone;       // 死区大小
} PID_Controller;




// // 编码器结构体
// typedef struct {

//     float speed_filter;         // 速度滤波值

// } Encoder_t;

// 电机控制结构体
typedef struct {
    // Encoder_t encoder;          // 编码器
    PID_Controller pid;         // PID内环控制器
	PID_Controller outerpid;    // PID外环控制器
    volatile float target_speed;         // 目标速度（RPM）
    volatile float current_speed;        // 当前速度（RPM）
    volatile float last_speed ;           // 上一次的速度（RPM）
	float target_quanshu;		//目标圈数    注意！，100=1圈
	volatile float current_quanshu;		//(当前圈数)   注意！，100=1圈
    volatile float last_quanshu ;        //(上一次的圈数) 注意！，100=1圈
    int16_t pwm_output;         // PWM输出值
    uint8_t enabled;            // 是否启用PID控制
} Motor_t;




// 在 Motor_t 结构体定义之后添加
extern Motor_t left_motor;
extern Motor_t right_motor;




void PID_Init(PID_Controller *pid, float kp, float ki, float kd,float output_limit,float integral_limit,float dead_zone);
float PID_Compute(PID_Controller *pid, float input);
void PID_SetSetpoint(PID_Controller *pid, float setpoint);
void PID_Reset(PID_Controller *pid);
void Motor_PID_Speed_Control(Motor_t *motor, float target_speed);
void Motor_PID_Position_Control(Motor_t *motor, float target_quanshu);
void Motor_Control_Task(void);
// void Encoder_Update(void);
void Motor_PID_Init(void);
void Set_Control_Mode(ControlMode_t mode);
void Reset_Encoder_Revolution_Count(void);
void Motor_Right_SetSpeed(uint8_t duty_percent);
void Motor_Left_SetSpeed(uint8_t duty_percent);
void TB6612_SetMotor(uint8_t motor_id, int16_t speed);
void Set_Motor_Target_Speed(float left_rpm, float right_rpm);
void Set_Motor_Target_Position(float left_quanshu, float right_quanshu);
uint8_t Is_Motion_Complete(void);


//动作
void Car_Go(float left_rpm, float right_rpm) ;
void Car_Go_Position(void) ;
void Car_Stop(void);
void Car_Stop_Precise(void);
void Car_Turn_Left_90_And_Stop(void);
void Car_Turn_Right_90_And_Stop(void);
void Car_Turn_Around_And_Stop(void);
void Car_Straight_Then_Turn_Right_90_And_Stop(void);
void Car_Straight_Then_Turn_Left_90_And_Stop(void);
uint8_t Is_Position_Control_Complete(void);
#endif  /* _BSP_TB6612_H */