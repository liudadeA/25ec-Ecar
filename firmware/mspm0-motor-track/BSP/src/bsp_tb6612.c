
#include "bsp_tb6612.h"
#include "board.h"
#include "stdlib.h"

float interval_in;
float interval_out;
uint8_t pid_control_enabled = 0;
static ControlMode_t current_control_mode = CONTROL_MODE_SPEED;
static uint8_t motion_complete = 1; // 动作完成标志
Motor_t left_motor = {0};   // 只在这里定义
Motor_t right_motor = {0};
float total_revolutions_right;
float total_revolutions_left;

/******************************************************************
 * 函 数 名 称：TB6612_Motor_Stop
 * 函 数 说 明：A端和B端电机停止（利用的是关闭电机的停止）
 * 函 数 形 参：无
 * 函 数 返 回：无
 * 作       者：LCKFB
 * 备       注：无
******************************************************************/
void TB6612_Motor_Stop(void)
{
    AIN1_OUT(1);
    AIN2_OUT(1);
    BIN1_OUT(1);
    BIN2_OUT(1);
}






//以下是自己写的
//初始化PID
void PID_Init(PID_Controller *pid, float kp, float ki, float kd,float output_limit,float integral_limit,float dead_zone)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    
    pid->setpoint = 0.0f;
    pid->input = 0.0f;
    pid->output = 0.0f;
    
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
    pid->derivative = 0.0f;
    
    pid->output_min = -output_limit;
    pid->output_max = output_limit;
    
    pid->last_time = HAL_GetTick;
    pid->enabled = 1;
    
    pid->integral_limit = integral_limit;  // 积分限制
    pid->dead_zone = dead_zone;        // 死区
}
//PID计算
float PID_Compute(PID_Controller *pid, float input)
{

    
    // 计算误差
    pid->input = input;
    pid->error = pid->setpoint - pid->input;
    
    // 死区处理
    if (fabs(pid->error) < pid->dead_zone) {
        pid->error = 0.0f;
    }
    
    // 比例项
    float proportional = pid->Kp * pid->error;
    
    // 积分项
    if (fabs(pid->error) > 0.1f) {
        pid->integral += pid->error ;
        
        // 积分限幅
        if (pid->integral > pid->integral_limit) {
            pid->integral = pid->integral_limit;
        } else if (pid->integral < -pid->integral_limit) {
            pid->integral = -pid->integral_limit;
        }
    }
    
    float integral_term = pid->Ki * pid->integral;
    
    // 微分项
    pid->derivative = (pid->error - pid->last_error) ;
    float derivative_term = pid->Kd * pid->derivative;
    
    // PID输出
    pid->output = proportional + integral_term + derivative_term;
    
    // 输出限幅
    if (pid->output > pid->output_max) {
        pid->output = pid->output_max;
    } else if (pid->output < pid->output_min) {
        pid->output = pid->output_min;
    }
    
    // 更新历史值
    pid->last_error = pid->error;

    
    return pid->output;
}   
//设置PID目标值
void PID_SetSetpoint(PID_Controller *pid, float setpoint)
{
    pid->setpoint = setpoint;
}
//重置PID控制器
void PID_Reset(PID_Controller *pid)
{
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
    pid->derivative = 0.0f;
    pid->output = 0.0f;
    pid->last_time = HAL_GetTick;
}


//更新编码器数据
//void Encoder_Update(void)
// {
//         left_motor.current_quanshu = left_count/10234*100;
//         right_motor.current_quanshu = right_count/10234*100;
//         // 计算转速（RPM），单位是转/分钟
//         left_motor.current_speed = (left_motor.current_quanshu-left_motor.last_quanshu)*60.0f*1000.0f/10.0f/100.0f;   //  每分钟多少转
//         right_motor.current_speed =(right_motor.current_quanshu-right_motor.last_quanshu)*60.0f*1000.0f/10.0f/100.0f;        //  每分钟多少转
        
//         left_motor.last_quanshu = left_motor.current_quanshu ;
//         right_motor.last_quanshu = right_motor.current_quanshu;
//         left_motor.last_speed = left_motor.current_speed ;
//         right_motor.last_speed = right_motor.current_speed ;
// }

//初始化电机PID控制系统
void Motor_PID_Init(void)
{

    // 初始化PID控制器
    PID_Init(&left_motor.pid, 0.02f, 0.21f, 0.13f,40.0f,150.0f,5.0f);      //内环PID参数设置
    PID_Init(&right_motor.pid, 0.02f, 0.21f, 0.13f,40.0f,150.0f,5.0f);		//kp ki,kd, output_limit, integral_limit, dead_zone
	
	PID_Init(&left_motor.outerpid,10.0f,0.0f,0.01f,100.0f,10.0f,3.0f);		//外环PID参数设置
    PID_Init(&right_motor.outerpid,10.0f,0.0f,0.01f,100.0f,10.0f,3.0f);
    
    // 初始化电机参数
    left_motor.target_speed = 0;
    left_motor.current_speed = 0;
    left_motor.last_speed = 0;
	left_motor.target_quanshu = 0;
	left_motor.current_quanshu = 0;
    left_motor.last_quanshu = 0;
    left_motor.enabled = 0;
    
    right_motor.target_speed = 0;
    right_motor.current_speed = 0;
    right_motor.last_speed = 0;
	right_motor.target_quanshu = 0;
	right_motor.current_quanshu = 0;
    right_motor.last_quanshu = 0;
    right_motor.enabled = 0;
    
    pid_control_enabled = 1;
}


//电机PID速度控制
void Motor_PID_Speed_Control(Motor_t *motor, float target_speed)
{
    // 更新编码器数据
    //Encoder_Update();
    
    // 设置目标速度
    motor->target_speed = target_speed;
        
    // 计算PID输出
    PID_SetSetpoint(&motor->pid, target_speed);
    float pid_output = PID_Compute(&motor->pid, motor->current_speed);
    
    motor->pwm_output = (int16_t)pid_output;
}

// 电机位置PID控制(为外环PID控制，目的是为了内环速度pid提供目标速度)
void Motor_PID_Position_Control(Motor_t *motor, float target_quanshu)
{
    // 更新编码器数据
    //Encoder_Update();
    
    // 设置目标圈数
    motor->target_quanshu = target_quanshu;

        
    // 计算PID输出
    PID_SetSetpoint(&motor->outerpid, target_quanshu);
    float pid_output = PID_Compute(&motor->outerpid, motor->current_quanshu);
    
    motor->target_speed = (int16_t)pid_output;
}



//统一的电机控制任务 - 根据当前模式选择控制方式
void Motor_Control_Task(void)
{
    static uint32_t last_control_time_in = 0;
    static uint32_t last_control_time_out = 0;
    uint32_t current_time = HAL_GetTick;
    
    if (!pid_control_enabled) return;
    
    if (current_control_mode == CONTROL_MODE_SPEED) {
        // 速度控制模式 - 每30ms执行一次
        interval_in = current_time - last_control_time_in;
        if (interval_in >= 30) {
            Motor_PID_Speed_Control(&left_motor, left_motor.target_speed);
            Motor_PID_Speed_Control(&right_motor, right_motor.target_speed);
            
            TB6612_SetMotor(0, left_motor.pwm_output);
            TB6612_SetMotor(1, right_motor.pwm_output);
            
            last_control_time_in = current_time;
        }
    } else {
        // 位置控制模式 - 串级PID
        interval_in = current_time - last_control_time_in;
        interval_out = current_time - last_control_time_out;
        
        // 内环速度控制 - 每10ms
        if (interval_in >= 10) {
            Motor_PID_Speed_Control(&left_motor, left_motor.target_speed);
            Motor_PID_Speed_Control(&right_motor, right_motor.target_speed);
            TB6612_SetMotor(0, left_motor.pwm_output);
            TB6612_SetMotor(1, right_motor.pwm_output);
            last_control_time_in = current_time;
        }
        
        // 外环位置控制 - 每40ms
        if (interval_out >= 40) {
            Motor_PID_Position_Control(&left_motor, left_motor.target_quanshu);
            Motor_PID_Position_Control(&right_motor, right_motor.target_quanshu);
            last_control_time_out = current_time;
        }
    }
}


//设置控制模式
void Set_Control_Mode(ControlMode_t mode)
{
    if (current_control_mode != mode) {
        current_control_mode = mode;
        // 切换模式时重置PID
        PID_Reset(&left_motor.pid);
        PID_Reset(&right_motor.pid);
        if (mode == CONTROL_MODE_POSITION) {
            PID_Reset(&left_motor.outerpid);
            PID_Reset(&right_motor.outerpid);
        }
    }
}

//重置编码器圈数计数
void Reset_Encoder_Revolution_Count(void)
{
    left_count = 0.0f;
    right_count = 0.0f;
}







// 设置右轮PWM占空比
// duty_percent: 占空比（0-100，对应0%-100%） 
void Motor_Right_SetSpeed(uint8_t duty_percent)
{
    uint16_t pulse = (duty_percent * (1000)) / 100;
    if (pulse > 999) pulse = 1000;  
    DL_TimerG_setCaptureCompareValue(PWM_0_INST, pulse , GPIO_PWM_0_C0_IDX);
    
}


// 设置左轮PWM占空比
// duty_percent: 占空比（0-100，对应0%-100%）
void Motor_Left_SetSpeed(uint8_t duty_percent)
{
    uint16_t pulse = (duty_percent * (1000)) / 100;
    if (pulse > 999) pulse = 1000;  
    DL_TimerG_setCaptureCompareValue(PWM_0_INST, pulse, GPIO_PWM_0_C1_IDX);
}


// TB6612电机驱动函数
void TB6612_SetMotor(uint8_t motor_id, int16_t speed)
{
    uint32_t pwm_value;
    
    // 限制速度范围
    if (speed > 100) speed = 100;
    if (speed < -100) speed = -100;
    
    // 计算PWM值
    pwm_value = (uint32_t)(abs(speed));
    
    if (motor_id == 0) {  // 左轮
        if (speed > 0) {
            // 正转
            AIN1_OUT(1);
            AIN2_OUT(0);
        } else if (speed <= 0) {
            // 反转
            AIN1_OUT(0);
            AIN2_OUT(1);
        } 
        Motor_Left_SetSpeed(pwm_value);
    } else {  // 右轮
        if (speed > 0) {
            // 正转
            BIN1_OUT(1);
            BIN2_OUT(0);
        } else if (speed <= 0) {
            // 反转
            BIN1_OUT(0);
            BIN2_OUT(1);
        } 
        Motor_Right_SetSpeed(pwm_value);
    }
}


//设置电机目标速度，
void Set_Motor_Target_Speed(float left_rpm, float right_rpm)
{
    left_motor.target_speed = left_rpm;
    right_motor.target_speed = right_rpm;
}
//设置电机目标圈数
void Set_Motor_Target_Position(float left_quanshu, float right_quanshu)
{
    left_motor.target_quanshu = left_quanshu;
    right_motor.target_quanshu = right_quanshu;
}

//检查运动是否完成
uint8_t Is_Motion_Complete(void)
{
    return motion_complete;
}

//连续运动 - 速度控制

void Car_Go(float left_rpm, float right_rpm)
{
    Set_Control_Mode(CONTROL_MODE_SPEED);
    Set_Motor_Target_Speed(left_rpm, right_rpm);
    motion_complete = 0; // 开始运动
    Set_Motor_Target_Position(left_motor.current_quanshu, 
                            right_motor.current_quanshu);
}

void Car_Go_Position(void)
{
    Set_Control_Mode(CONTROL_MODE_POSITION);
    float current_left = left_motor.current_quanshu;
    float current_right = right_motor.current_quanshu;
    Set_Motor_Target_Position(current_left + 150.0f, current_right - 150.0f);
    
}



//立即停止 - 速度控制
void Car_Stop(void)
{
    Set_Control_Mode(CONTROL_MODE_SPEED);
    Set_Motor_Target_Speed(0.0f, 0.0f);
    motion_complete = 1; // 停止完成
}


//精确停止 - 位置控制
void Car_Stop_Precise(void)
{
    Set_Control_Mode(CONTROL_MODE_POSITION);
    // 保持当前位置
    Set_Motor_Target_Position(left_motor.current_quanshu, 
                            right_motor.current_quanshu);
    motion_complete = 1;
}

//原地左转90度
void Car_Turn_Left_90_And_Stop(void)
{
    static uint8_t turn_initialized = 0;
    
    if (!turn_initialized) {
        //Car_Stop_Precise();
        //Motor_Control_Task();
        // total_revolutions_left = left_motor.current_quanshu;
        // total_revolutions_right = right_motor.current_quanshu;
        
        // // 左转：左轮后退，右轮前进
        // Set_Motor_Target_Position(total_revolutions_left , 
        //                         total_revolutions_right + 160.0f);
        // left_motor.target_quanshu = left_quanshu;
        right_motor.target_quanshu += 160.0f;
        Set_Control_Mode(CONTROL_MODE_POSITION);
        turn_initialized = 1;
        motion_complete = 0;
    }
    
    if (Is_Position_Control_Complete()) 
    {
        turn_initialized = 0;
        motion_complete = 1;
    }
}





//原地右转90度
void Car_Turn_Right_90_And_Stop(void)
{
    static uint8_t turn_initialized = 0;
    
    if (!turn_initialized) {
        Set_Control_Mode(CONTROL_MODE_POSITION);
        float total_revolutions_left = left_motor.current_quanshu;
        float total_revolutions_right = right_motor.current_quanshu;
        
        // 右转：左轮前进，右轮后退
        Set_Motor_Target_Position(total_revolutions_left + 160.0f, 
                                total_revolutions_right );
        turn_initialized = 1;
        motion_complete = 0;
    }
    
    if (Is_Position_Control_Complete()) {
        turn_initialized = 0;
        motion_complete = 1;
    }
}



//原地掉头180度

void Car_Turn_Around_And_Stop(void)
{
    static uint8_t turn_initialized = 0;
    
    if (!turn_initialized) {
        Set_Control_Mode(CONTROL_MODE_POSITION);
        total_revolutions_left = left_motor.current_quanshu;
        total_revolutions_right = right_motor.current_quanshu;
        
        // 掉头：两轮反向转动
        Set_Motor_Target_Position(total_revolutions_left + 150.0f, 
                                total_revolutions_right - 150.0f);
        turn_initialized = 1;
        motion_complete = 0;
    }
    
    if (Is_Position_Control_Complete()) {
        turn_initialized = 0;
        motion_complete = 1;
    }
}

//先直行再右转90度
void Car_Straight_Then_Turn_Right_90_And_Stop(void)
{
    static uint8_t step = 0;
    static uint8_t initialized = 0;
    
    if (!initialized) {
        Set_Control_Mode(CONTROL_MODE_POSITION);
        total_revolutions_left = left_motor.current_quanshu;
        total_revolutions_right = right_motor.current_quanshu;
        step = 0;
        initialized = 1;
        motion_complete = 0;
    }
    
    if (step == 0) {
        // 第一步：直行
        Set_Motor_Target_Position(total_revolutions_left + 150.0f, 
                                total_revolutions_right + 150.0f);
        
        if (Is_Position_Control_Complete()) {
            step = 1;
            // 记录转向起点
            total_revolutions_left = left_motor.current_quanshu;
            total_revolutions_right = right_motor.current_quanshu;
        }
    } else if (step == 1) {
        // 第二步：右转90度
        Set_Motor_Target_Position(total_revolutions_left + 160.0f, 
                                total_revolutions_right );
        
        if (Is_Position_Control_Complete()) {
            initialized = 0;
            step = 0;
            motion_complete = 1;
        }
    }
}

//先直行再左转90度
void Car_Straight_Then_Turn_Left_90_And_Stop(void)
{
    static uint8_t step = 0;
    static uint8_t initialized = 0;
    
    if (!initialized) {
        Set_Control_Mode(CONTROL_MODE_POSITION);
        float total_revolutions_left = left_motor.current_quanshu;
        float total_revolutions_right = right_motor.current_quanshu;
        step = 0;
        initialized = 1;
        motion_complete = 0;
    }
    
    if (step == 0) {
        // 第一步：直行
        Set_Motor_Target_Position(total_revolutions_left + 150.0f, 
                                total_revolutions_right + 150.0f);
        
        if (Is_Position_Control_Complete()) {
            step = 1;
            // 记录转向起点
            total_revolutions_left = left_motor.current_quanshu;
            total_revolutions_right = right_motor.current_quanshu;
        }
    } else if (step == 1) {
        // 第二步：左转90度
        Set_Motor_Target_Position(total_revolutions_left , 
                                total_revolutions_right + 160.0f);
        
        // if (Is_Position_Control_Complete()) {
        //     initialized = 0;
        //     step = 0;
        //     motion_complete = 1;
        // }
    }
}


//检查位置控制是否完成
uint8_t Is_Position_Control_Complete(void)
{
    if (current_control_mode != CONTROL_MODE_POSITION) {
        return 1; // 速度模式总是返回完成
    }
    lc_printf("\xBB[电机] 左轮圈数: %.2f, 右轮圈数: %.2f\r\n\x55\x44\x33", 
           left_motor.current_quanshu, right_motor.current_quanshu);
          


    lc_printf("\xBB[电机] 左轮圈数: %.2f, 右轮圈数: %.2f\r\n\x55\x44\x33", 
           left_motor.current_quanshu, right_motor.current_quanshu);

    float left_error = fabs(left_motor.target_quanshu - left_motor.current_quanshu);
    float right_error = fabs(right_motor.target_quanshu - right_motor.current_quanshu);
    
    lc_printf("\xBB[电机] 左轮误差: %.2f, 右轮误差: %.2f\r\n\x55\x44\x33", 
           left_error, right_error);

    return (left_error <= 10.0f && right_error <= 10.0f);
}