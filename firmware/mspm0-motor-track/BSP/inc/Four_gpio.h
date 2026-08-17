#ifndef ___Four_GPIO_H
#define ___Four_GPIO_H

#include "ti_msp_dl_config.h"
#include <string.h>

// 传感器采样时间
#define Sensors_Delay 10

#define SENSOR_1  (1 << 3)  // 左
#define SENSOR_2  (1 << 2)
#define SENSOR_3  (1 << 1)
#define SENSOR_4  (1 << 0)  // 右



// ================4 - channel key state macros ==================
// 1 | 2 | 3 | 4
/*有线为1，无线为0*/
#define STATE_STRAIGHT        (SENSOR_2 | SENSOR_3)
#define STATE_ADJUST_LEFT     (SENSOR_2)
#define STATE_ADJUST_RIGHT    (SENSOR_3)
#define STATE_CURVE_LEFT      (SENSOR_1)
#define STATE_CURVE_RIGHT     (SENSOR_4)
#define STATE_CROSS_ROAD_1    (SENSOR_1 | SENSOR_2 | SENSOR_3)
#define STATE_CROSS_ROAD_2    (SENSOR_1 | SENSOR_2)
#define STATE_OVER             (0x00)
// 传感器结构体
typedef struct {
    uint8_t DH1;
    uint8_t DH2;
    uint8_t DH3;
    uint8_t DH4;
	uint8_t DH;
} Sensor4_State;

typedef struct {
    uint8_t DH1;
    uint8_t DH2;
    uint8_t DH3;
    uint8_t DH4;
    uint8_t DH5;
    uint8_t DH6;
    uint8_t DH7;
    uint8_t DH8;
    uint8_t DH_TURN1;
    uint8_t DH_TURN2;
	uint8_t DH;
    uint8_t state;
} Sensor8_State;
// void GPIO_Get_4Digital(Sensor4_State *state);
void GPIO_Get_8Digital(Sensor8_State *state);
void Four_LineTrack(uint8_t DH);
void Eight_LineTrack(uint8_t sensor_state_8);


#endif