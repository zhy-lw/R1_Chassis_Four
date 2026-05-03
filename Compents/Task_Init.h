#ifndef __TASK_INIT_H
#define __TASK_INIT_H

#include "drive_callback.h"
#include "ForceChassis.h"
#include "FreeRTOS.h"
#include "task.h"
#include "usb_device.h"
#include "motorEx.h"

#define M_PI 3.1415926f
#define MAX_ROBOT_VEL 3.5f //	 m/s
#define MAX_ROBOT_OMEGA ANGLE2RAD(80.0f)

void Task_Init(void);

typedef struct{
	uint8_t Left_Key_Up;         
	uint8_t Left_Key_Down;       
	uint8_t Left_Key_Left;       
	uint8_t Left_Key_Right;       
	uint8_t Left_Switch_Up;       
	uint8_t Left_Switch_Down;
	uint8_t Left_Broadside_Key;

	uint8_t Right_Key_Up;        
	uint8_t Right_Key_Down;      
	uint8_t Right_Key_Left;      
	uint8_t Right_Key_Right;     
	uint8_t Right_Switch_Up;      
	uint8_t Right_Switch_Down;      
	uint8_t Right_Broadside_Key;
} hw_key_t;

typedef struct {
    float Ex;
    float Ey;
    float Eomega;
    hw_key_t First,Second;
} Remote_Handle_t;

typedef enum{
    STOP,
    REMOTE,
    AUTO,
		STRETCH
}ChassisMode;

#pragma pack(1)
typedef struct
{
    uint8_t head;
    float expectDirection[2];
    float expextVelocity[2];
    uint8_t tail;
		uint16_t crc;
} Pack_TransRemote_t;
#pragma pack()

#pragma pack(1)
typedef struct {
    uint16_t spi1;
    uint16_t spi2;
    uint16_t spi3;
    uint16_t adc;
} LASER_SEND_Typedef;   //激光句柄
#pragma pack()

#endif
