#include "Four_gpio.h"
#include "bsp_tb6612.h"
#include "ti_msp_dl_config.h"

/***************************************
4路循迹读取代码
1. 灰度传感器使用[上拉输入]，跳线帽接pull  1：指示灯亮 0：指示灯灭 
2. 红外传感器使用[下拉输入]    1:灭（白）  0：亮（黑线）
****************************************/
// void GPIO_Get_4Digital(Sensor4_State *state) {

//         state->DH1 = (DL_GPIO_readPins(GPIO_GRP_Sensor_PORT, GPIO_GRP_Sensor_DH_1_PIN))? 1:0; //最左
//         state->DH2 = (DL_GPIO_readPins(GPIO_GRP_Sensor_PORT, GPIO_GRP_Sensor_DH_2_PIN))? 1:0;
//         state->DH3 = (DL_GPIO_readPins(GPIO_GRP_Sensor_PORT, GPIO_GRP_Sensor_DH_3_PIN))? 1:0; 
//         state->DH4 = (DL_GPIO_readPins(GPIO_GRP_Sensor_PORT, GPIO_GRP_Sensor_DH_4_PIN))? 1:0; //最右

//         // 使用0000 （左1...右1）格式
//         state->DH = (state->DH1<<3) | (state->DH2<<2) | (state->DH3<<1) | (state->DH4<<0);
// 		state->DH = ~state->DH; //
// 		state->DH &= 0x0F;       //
//         //lc_printf("Raw: DH1=%d, DH2=%d, DH3=%d, DH4=%d\r\n", 
//     //state->DH1, state->DH2, state->DH3, state->DH4);
//         lc_printf("state:%d-%d-%d-%d\r\n",(state->DH>>3) & 0x01,(state->DH>>2)& 0x01,(state->DH>>1)& 0x01,(state->DH>>0)& 0x01);
// }

// 八路读取数字量 0(灭) 黑线 1（亮）白
void GPIO_Get_8Digital(Sensor8_State *state) {

        state->DH1 = (DL_GPIO_readPins(GPIOA, GPIO_GRP_Sensor_DH_1_PIN))? 1:0; //最左
        state->DH2 = (DL_GPIO_readPins(GPIOA, GPIO_GRP_Sensor_DH_2_PIN))? 1:0; 
        state->DH3 = (DL_GPIO_readPins(GPIOA, GPIO_GRP_Sensor_DH_3_PIN))? 1:0;
        state->DH4 = (DL_GPIO_readPins(GPIOA, GPIO_GRP_Sensor_DH_4_PIN))? 1:0;
        state->DH5 = (DL_GPIO_readPins(GPIOB, GPIO_GRP_Sensor_DH_5_PIN))? 1:0;
        state->DH6 = (DL_GPIO_readPins(GPIOB, GPIO_GRP_Sensor_DH_6_PIN))? 1:0;
        state->DH7 = (DL_GPIO_readPins(GPIOA, GPIO_GRP_Sensor_DH_7_PIN))? 1:0;
        state->DH8 = (DL_GPIO_readPins(GPIOB, GPIO_GRP_Sensor_DH_8_PIN))? 1:0; //最右

        state->DH_TURN1 = (DL_GPIO_readPins(GPIOB, GPIO_GRP_Sensor_DH_TURN_1_PIN))? 1:0; //
        state->DH_TURN2 = (DL_GPIO_readPins(GPIOB, GPIO_GRP_Sensor_DH_TURN_2_PIN))? 1:0; //
        // 使用0000 （左1...右1）格式
        state->DH = (state->DH1<<7) | (state->DH2<<6) | (state->DH3<<5) | (state->DH4<<4) | (state->DH5<<3) | (state->DH6<<2) | (state->DH7<<1) | (state->DH8<<0) ;
		//state->DH = ~state->DH; //
        //lc_printf("Raw: DH1=%d, DH2=%d, DH3=%d, DH4=%d\r\n", 
    //state->DH1, state->DH2, state->DH3, state->DH4);
        lc_printf("state:%d-%d-%d-%d-%d-%d-%d-%d\r\n",(state->DH>>7) & 0x01,(state->DH>>6)& 0x01,(state->DH>>5)& 0x01,(state->DH>>4)& 0x01,(state->DH>>3) & 0x01,(state->DH>>2)& 0x01,(state->DH>>1)& 0x01,(state->DH>>0)& 0x01);
}
//========================= 8路循迹转换 =======================//
// 传感器编号【左高位】 0   1     2     3 4     5      6 7 【右低位】
// 获取最靠外触发的传感器编号（1-8），无触发返回0
void get_outermost_sensor(Sensor8_State *sensor_8) {
    int leftmost = -1;  // 最左侧触发的传感器编号
    int rightmost = -1; // 最右侧触发的传感器编号
    // 从左向右扫描：找最左侧触发的传感器
    for (int i = 0; i < 8; i++) {
        if (sensor_8->DH & (1 << i)) { // 检查第i位是否触发
            leftmost = i;
            break; // 找到最左侧后退出
        }
    }

    // 从右向左扫描：找最右侧触发的传感器
    for (int i = 7; i >= 0; i--) {
        if (sensor_8->DH & (1 << i)) { 
            rightmost = i;
            break; // 找到最右侧后退出
        }
    }

    // 判断最靠外的传感器
    if (leftmost == -1 && rightmost == -1) 
    {
        sensor_8->state = 0; // 无触发
        return;
    }
    if (leftmost == -1) 
    {
        sensor_8->state =  rightmost + 1;             // 仅右侧触发
        return;
    }
    if (rightmost == -1)
    {
        sensor_8->state =  leftmost + 1;             // 仅左侧触发
        return;
    }

    if((sensor_8->DH_TURN2 + sensor_8->DH_TURN1 + sensor_8->DH2 + sensor_8->DH1) >= 3)
    {
        sensor_8->state = 10;
        return;
    }
    sensor_8->state = (leftmost < (7 - rightmost)) ? (leftmost + 1) : (rightmost + 1);
}
//=================8 路循迹状态 ==========================
/*传感器编号【左高位】 1   2     3   4  5     6    7 8 【右低位】
   状态             左大弯 | 左微调| 直走| 右微调|  右大弯
		四个传感器为1：岔路口 
		全都为0：丢线
*/
/*
void Car_Go(float left_rpm, float right_rpm) ;
void Car_Go_Position(void) ;
void Car_Stop(void);
void Car_Stop_Precise(void);
void Car_Turn_Left_90_And_Stop(void);
void Car_Turn_Right_90_And_Stop(void);
void Car_Turn_Around_And_Stop(void);
void Car_Straight_Then_Turn_Right_90_And_Stop(void);
void Car_Straight_Then_Turn_Left_90_And_Stop(void);
*/
//纯速度转向
extern volatile uint8_t last_state_main;
extern uint8_t Turn_Time;
extern volatile uint8_t Cross_Counter;
extern uint8_t taking_N;
extern volatile uint32_t NO_Cross_time;
extern volatile uint32_t HAL_GetTick;
void Four_LineTrack(uint8_t DH) {
    switch(DH) {
        //case STATE_STRAIGHT:
        case 0x06:
            Car_Go(150.0f,150.0f);
            Motor_Control_Task();
            lc_printf("go straight\r\n");
            break;

        //case STATE_ADJUST_LEFT:
        case 0x04:
            Car_Go(140.0f,160.0f);
            Motor_Control_Task();
            lc_printf("adjust left\r\n");
            break;

        //case STATE_ADJUST_RIGHT:
        case 0x02:
            Car_Go(160.0f,140.0f);
            Motor_Control_Task();
            lc_printf("adjust right\r\n");
            break;

        //case STATE_CURVE_LEFT:
        case 0x08:
            Car_Go(140.0f,160.0f);
            Motor_Control_Task();
            lc_printf("curve left\r\n");
            break;

        //case STATE_CURVE_RIGHT:
        case 0x01:
            Car_Go(155.0f,145.0f);
            Motor_Control_Task();
            lc_printf("curve right\r\n");
            break;

        //case STATE_CROSS_ROAD_1:   // 十字路口类型1 左转
        case 0x0E:
        case 0x0C:
        case 0x0F:
        /*
            Car_Stop_Precise();
            while(1){
                Motor_Control_Task();
            }
        */
        {
            Cross_Counter++;
            if(Cross_Counter <= 4*taking_N)
            {
                
                Car_Stop_Precise();
                while(Turn_Time < 5)
                {
                    if(casee_change())
                    {
                        Motor_Control_Task();
                        Turn_Time++;
                    }          
                }
                Turn_Time = 0;
                Car_Go(-40.0f,50.0f);
                while(Turn_Time <85 ){
                    if(casee_change())
                    {
                        Motor_Control_Task();
                        Turn_Time++;
                    }
                }
                Turn_Time = 0;
                NO_Cross_time = HAL_GetTick;
                lc_printf("cross road detected -->> turn left\r\n");
            }
            else {  //任务结束
                Car_Stop_Precise();
                while(1){
                    if(casee_change())
                    {
                        Motor_Control_Task();
                    }
                }
            }
        }
        break;
        //case STATE_OVER:
        case 0x00:
            if(last_state_main == 0x08){ //左大弯丢线
                Car_Go(140.0f,160.0f);
                Motor_Control_Task();
                break;
            }
            if(last_state_main == 0x01){ // 右大弯丢线
                Car_Go(160.0f,140.0f);
                Motor_Control_Task();
                break;
            }
            if(last_state_main == 0x04){ // 左微调丢线
                Car_Go(142.0f,158.0f);
                Motor_Control_Task();
                break;
            }
            if(last_state_main == 0x01){ // 右微调丢线
                Car_Go(158.0f,142.0f);
                Motor_Control_Task();
                break;
            }
            //Car_Go(150.0f,150.0f);
            //Motor_Control_Task();
            //lc_printf("line out -->> going\r\n");
            break;
        default:  // 异常状态处理
            lc_printf("error: unknown state %d\r\n", DH);
            break;
    }
    last_state_main = DH;
    //lc_printf("%.2f,%.2f\r\n",left_motor.current_quanshu,right_motor.current_quanshu);
    // lc_printf("%.2f,%.2f\r\n",left_motor.current_speed,right_motor.current_speed);
    //lc_printf("=======================================\r\n");
}

void Eight_LineTrack(uint8_t sensor_state_8)
{
		    // 标准循线逻辑
    switch(sensor_state_8) {
        case 4: //直走
		case 5: //直走
        {
            Car_Go(150.0f,150.0f);
            Motor_Control_Task();
            lc_printf("go straight\r\n");
            break;
        }    
        case 3: // 左微调
            Car_Go(140.0f,160.0f);
            Motor_Control_Task();
            lc_printf("go straight\r\n");
            break;
            
        case 6: //右微调
            Car_Go(160.0f,140.0f);
            Motor_Control_Task();
            lc_printf("go straight\r\n");
            break;
            
        case 2: //左大弯
            Car_Go(142.0f,158.0f);
            Motor_Control_Task();
            lc_printf("go straight\r\n");
            break;
		case 1: // 左大大弯
            Car_Go(175.0f,125.0f);
            Motor_Control_Task();
            lc_printf("go straight\r\n");
            break;
            
        case 7: //右大弯 
            Car_Go(158.0f,142.0f);
            Motor_Control_Task();
            lc_printf("go straight\r\n");
            break;
		case 8: //右大大弯
            Car_Go(125.0f,175.0f);
            Motor_Control_Task();
            lc_printf("go straight\r\n");
            break;
            
        case 10:  // 十字、T字路口
        {
            Cross_Counter++;
            if(Cross_Counter <= 4*taking_N)
            {
                
                // Car_Stop();
                // while(Turn_Time < 20)
                // {
                //     if(casee_change())
                //     {
                //         Motor_Control_Task();
                //         Turn_Time++;
                //     }          
                // }
                Turn_Time = 0;
                Car_Go(0.0f,100.0f);
                while(Turn_Time < 85){
                    if(casee_change())
                    {
                        Motor_Control_Task();
                        Turn_Time++;
                    }
                }
                Turn_Time = 0;
                NO_Cross_time = HAL_GetTick;
                lc_printf("cross road detected -->> turn left\r\n");
            }
            else {  //任务结束
                Car_Stop_Precise();
                while(1){
                    if(casee_change())
                    {
                        Motor_Control_Task();
                    }
                }
            }
        }
        break;
        case 0:  // 丢线
            if(last_state_main == 1) //左丢线
            {
                Car_Go(180.0f,120.0f);
                Motor_Control_Task();
                break;
            }
            if(last_state_main == 8) // 右丢线
            {
                Car_Go(120.0f,180.0f);
                Motor_Control_Task();
                break;
            }
            if(last_state_main == 10)
            {
                Car_Go(120.0f,180.0f);
                Motor_Control_Task();
                break;               
            }
            break;
						
        default:
            Car_Go(150.0f,150.0f);
            Motor_Control_Task();
            break;
    }
    last_state_main = sensor_state_8;
}

// //  任务二巡线
// void Eight_LineTrack_2(uint8_t sensor_state_8)
// {
// 		    // 标准循线逻辑
//     switch(sensor_state_8) {
//         case 4: //直走
// 		case 5: //直走
//         {
//             Car_Go(150.0f,150.0f);
//             Motor_Control_Task();
//             lc_printf("go straight\r\n");
//             break;
//         }    
//         case 3: // 左微调
//             Car_Go(147.0f,152.0f);
//             Motor_Control_Task();
//             lc_printf("go straight\r\n");
//             break;
            
//         case 6: //右微调
//             Car_Go(152.0f,147.0f);
//             Motor_Control_Task();
//             lc_printf("go straight\r\n");
//             break;
            
//         case 2: //左大弯
//             Car_Go(145.0f,155.0f);
//             Motor_Control_Task();
//             lc_printf("go straight\r\n");
//             break;
// 		case 1: // 左大大弯
//             Car_Go(157.0f,142.0f);
//             Motor_Control_Task();
//             lc_printf("go straight\r\n");
//             break;
            
//         case 7: //右大弯 
//             Car_Go(155.0f,145.0f);
//             Motor_Control_Task();
//             lc_printf("go straight\r\n");
//             break;
// 		case 8: //右大大弯
//             Car_Go(142.0f,157.0f);
//             Motor_Control_Task();
//             lc_printf("go straight\r\n");
//             break;
            
//         case 10:  // 十字、T字路口
//         {
//             Cross_Counter++;
//             if(Cross_Counter <= 4*taking_N)
//             {
//                 //发送数据给stm32
//                 uart_send_packet(UART_Regs *uart, uint8_t *data, uint16_t length);

//                 Car_Stop();
//                 while(Turn_Time < 20)
//                 {
//                     if(casee_change())
//                     {
//                         Motor_Control_Task();
//                         Turn_Time++;
//                     }          
//                 }
//                 Turn_Time = 0;
//                 Car_Go(0.0f,100.0f);
//                 while(Turn_Time < 80){
//                     if(casee_change())
//                     {
//                         Motor_Control_Task();
//                         Turn_Time++;
//                     }
//                 }
//                 Turn_Time = 0;
//                 NO_Cross_time = HAL_GetTick;
//                 lc_printf("cross road detected -->> turn left\r\n");
//             }
//             else {  //任务结束
//                 Car_Stop_Precise();
//                 while(1){
//                     if(casee_change())
//                     {
//                         Motor_Control_Task();
//                     }
//                 }
//             }
//         }
//         break;
//         case 0:  // 丢线
//             if(last_state_main == 1) //左丢线
//             {
//                 Car_Go(170.0f,130.0f);
//                 Motor_Control_Task();
//                 break;
//             }
//             if(last_state_main == 8) // 右丢线
//             {
//                 Car_Go(130.0f,170.0f);
//                 Motor_Control_Task();
//                 break;
//             }
//             if(last_state_main == 10)
//             {
//                 Car_Go(120.0f,180.0f);
//                 Motor_Control_Task();
//                 break;               
//             }
//             break;
						
//         default:
//             Car_Go(150.0f,150.0f);
//             Motor_Control_Task();
//             break;
//     }
//     last_state_main = sensor_state_8;
// }