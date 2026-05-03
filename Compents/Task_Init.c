#include "Task_Init.h"
#include "VL53_100.h"
#include "encoder.h"
#include "usart.h"

#include "comm_stm32_hal_middle.h"
#include "dataFrame.h"
#include "comm.h"

#include "AutoPilot.h"
#include "Pilot_callback.h"
#include "Action_Config.h"
#include "Action.h"

#include "crc_ccitt.h"

SteeringWheel steeringWheelArray[4];
Wheel_t wheelArray[4];
Chassis_t chassis;

//句柄
TaskHandle_t Remote_Analysis_Handle;
TaskHandle_t Uart_Tx_Handle;
extern TaskHandle_t task_handle;

//遥控器数据
uint8_t usart4_dma_buff[60];
uint8_t usart5_dma_buff[60];
Remote_Handle_t Remote_Control;
extern SemaphoreHandle_t Remote_semaphore;

//距离传感器
LASER_SEND_Typedef DT_35_Len;

//任务
void Remote_Analysis_Task(void *pvParameters);
void Uart_Tx(void *pvParameters);

ChassisMode Mode = REMOTE;

void Task_Init(void)
{
    __HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart4, usart4_dma_buff, sizeof(usart4_dma_buff));
		//遥控器
		__HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart5, usart5_dma_buff, sizeof(usart5_dma_buff));
		__HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
	
    wheelArray[0].pos.x =  0.325f;
    wheelArray[0].pos.y =  0.325f; 
    wheelArray[0].pos.z =  - PI / 4.0f;
    wheelArray[1].pos.x =  0.325f;
    wheelArray[1].pos.y =  -0.325f;
    wheelArray[1].pos.z =  - PI / 4.0f;
    wheelArray[2].pos.x =  -0.325f;
    wheelArray[2].pos.y =  -0.325f;
    wheelArray[2].pos.z =  - PI/ 4.0f;
    wheelArray[3].pos.x =  -0.325f;
    wheelArray[3].pos.y =  0.325f;
    wheelArray[3].pos.z =  PI * 3.0 / 4.0f;

    for(int i = 0; i < 4; i++)
    {
        wheelArray[i].user_data = &steeringWheelArray[i];
        wheelArray[i].set_target_cb = SetWheelTarget_Callback;
        chassis.wheel[i] = &wheelArray[i];
    }
		
    Vector2D barycenter = {0, 0};
    ChassisInit(&chassis, wheelArray, 4, barycenter, 25.2f, 1.25f, 0.00001f, 2, 600, 4);
		xTaskCreate(Remote_Analysis_Task, "Remote_Analysis_Task", 128, NULL, 4, &Remote_Analysis_Handle);
		xTaskCreate(Uart_Tx, "Uart_Tx", 256, NULL, 3, &Uart_Tx_Handle);
}

PackControl_t recv_pack;
uint8_t recv_buff[20] = {0};
float rocker_filter[4] = {0};
static void Key_Parse(uint32_t key, hw_key_t *out)
{
    out->Right_Switch_Up     = (key & KEY_Right_Switch_Up)     ? 1 : 0;
    out->Right_Switch_Down   = (key & KEY_Right_Switch_Down)   ? 1 : 0;

    out->Right_Key_Up        = (key & KEY_Right_Key_Up)        ? 1 : 0;
    out->Right_Key_Down      = (key & KEY_Right_Key_Down)      ? 1 : 0;
    out->Right_Key_Left      = (key & KEY_Right_Key_Left)      ? 1 : 0;
    out->Right_Key_Right     = (key & KEY_Right_Key_Right)     ? 1 : 0;

    out->Right_Broadside_Key = (key & KEY_Right_Broadside_Key) ? 1 : 0;

    out->Left_Switch_Up      = (key & KEY_Left_Switch_Up)      ? 1 : 0;
    out->Left_Switch_Down    = (key & KEY_Left_Switch_Down)    ? 1 : 0;

    out->Left_Key_Up         = (key & KEY_Left_Key_Up)         ? 1 : 0;
    out->Left_Key_Down       = (key & KEY_Left_Key_Down)       ? 1 : 0;
    out->Left_Key_Left       = (key & KEY_Left_Key_Left)       ? 1 : 0;
    out->Left_Key_Right      = (key & KEY_Left_Key_Right)      ? 1 : 0;

    out->Left_Broadside_Key  = (key & KEY_Left_Broadside_Key)  ? 1 : 0;
}

void Remote_Analysis()
{
    if(xSemaphoreTake(Remote_semaphore, pdMS_TO_TICKS(200)) == pdTRUE)
    {
      /* 1. 保存上一帧 */
      Remote_Control.Second = Remote_Control.First;
			
      /* 2. 解析当前按键 */
      Key_Parse(recv_pack.Key, &Remote_Control.First);

      Remote_Control.Ex = - recv_pack.rocker[1] / 1597.0f *MAX_ROBOT_VEL;
      Remote_Control.Ey = recv_pack.rocker[0] / 1597.0f *MAX_ROBOT_VEL;
      Remote_Control.Eomega = recv_pack.rocker[2] / 1597.0f * MAX_ROBOT_OMEGA;
    }else {
      Remote_Control.Ex = 0;
      Remote_Control.Ey = 0;
      Remote_Control.Eomega = 0;

      memset(&Remote_Control.First, 0, sizeof(Remote_Control.First));
    }
}

void MyRecvCallback(uint8_t *src, uint16_t size, void *user_data)
{
  memcpy(&recv_buff, src, size);
  memcpy(&recv_pack, recv_buff, sizeof(recv_pack));
  xSemaphoreGive(Remote_semaphore);
}
CommPackRecv_Cb  recv_cb = MyRecvCallback;

void Remote_Analysis_Task(void *pvParameters)
{
	g_comm_handle = Comm_Init(&huart5);
	RemoteCommInit(NULL);
	register_comm_recv_cb(recv_cb, 0x01, &recv_pack);
	while(1)
	{
		Remote_Analysis();
	}
}

float expect_len = 217.0f;
PID2  One_Four_PID, Two_Three_PID;
Pack_TransRemote_t pack_t[2];
void Uart_Tx(void *pvParameters)
{
  TickType_t last_wake_time = xTaskGetTickCount();
	pack_t[0].head = 0xAB;
	pack_t[0].tail = 0xBA;
	
	pack_t[1].head = 0xAB;
	pack_t[1].tail = 0xBA;
	
	One_Four_PID.Kp = 0.001f;
	One_Four_PID.Ki = 0.0f;
	One_Four_PID.Kd = 0.0f;
	One_Four_PID.limit = 10.0f;
	One_Four_PID.output_limit = 0.2f;
	
	Two_Three_PID.Kp = 0.005f;
	Two_Three_PID.Ki = 0.0f;
	Two_Three_PID.Kd = 0.0f;
	Two_Three_PID.limit = 10.0f;
	Two_Three_PID.output_limit = 0.5f;
	
  while(1)
  {
		pack_t[0].expectDirection[0] = steeringWheelArray[0].expectDirection;//1和4
		pack_t[0].expectDirection[1] = steeringWheelArray[3].expectDirection;
		pack_t[0].expextVelocity[0] = steeringWheelArray[0].expextVelocity;
		pack_t[0].expextVelocity[1] = steeringWheelArray[3].expextVelocity;
		
		pack_t[1].expectDirection[0] = steeringWheelArray[1].expectDirection;//2和3
		pack_t[1].expectDirection[1] = steeringWheelArray[2].expectDirection;
		pack_t[1].expextVelocity[0] = steeringWheelArray[1].expextVelocity;
		pack_t[1].expextVelocity[1] = steeringWheelArray[2].expextVelocity;
		
		chassis.exp_vel.x = Remote_Control.Ex;
		chassis.exp_vel.y = Remote_Control.Ey;
		chassis.exp_vel.z = Remote_Control.Eomega;
		
    pack_t[0].crc = crc_ccitt(0, (uint8_t *)&pack_t[0], sizeof(Pack_TransRemote_t)-2);
		pack_t[1].crc = crc_ccitt(0, (uint8_t *)&pack_t[1], sizeof(Pack_TransRemote_t)-2);
		HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&pack_t[0], sizeof(Pack_TransRemote_t));
		HAL_UART_Transmit_DMA(&huart6, (uint8_t *)&pack_t[1], sizeof(Pack_TransRemote_t));
		
		if(Remote_Control.First.Right_Key_Up == 1 && Remote_Control.Second.Right_Key_Up == 0)
		{
			steeringWheelArray[0].expectDirection = -45.0f;
			steeringWheelArray[3].expectDirection = 135.0f;//向左
			
			steeringWheelArray[1].expectDirection = 135.0f;
			steeringWheelArray[2].expectDirection = 135.0f;//向右
			Mode = STRETCH;
			expect_len = 500;
			vTaskSuspend(task_handle);
		}
		
		if(Remote_Control.First.Right_Key_Down == 1 && Remote_Control.Second.Right_Key_Down == 0)
		{
			steeringWheelArray[0].expectDirection = -45.0f;
			steeringWheelArray[3].expectDirection = 135.0f;//向左
			
			steeringWheelArray[1].expectDirection = 135.0f;
			steeringWheelArray[2].expectDirection = 135.0f;//向右
			Mode = STRETCH;
			expect_len = 220;
			vTaskSuspend(task_handle);
		}
		
		if(Remote_Control.First.Left_Key_Up == 1 && Remote_Control.Second.Left_Key_Up == 0)
		{
			Mode = REMOTE;
			vTaskResume(task_handle);
		}
		
		if(Mode == REMOTE)
		{
			wheelArray[0].pos.x =  0.325f;
			wheelArray[0].pos.y =  0.325f + (DT_35_Len.spi2 - 217.0f) / 1000.0f;
			
			wheelArray[1].pos.x =  0.325f;
			wheelArray[1].pos.y =  -(0.325f + (DT_35_Len.spi2 - 217.0f) / 1000.0f);
			
			wheelArray[2].pos.x =  -0.325f;
			wheelArray[2].pos.y =  -(0.325f + (DT_35_Len.spi2 - 217.0f) / 1000.0f);

			wheelArray[3].pos.x =  -0.325f;
			wheelArray[3].pos.y =  0.325f + (DT_35_Len.spi2 - 217.0f) / 1000.0f;
		}
		
		if(Mode == STRETCH)
		{
			PID_Control2(DT_35_Len.spi2, expect_len, &One_Four_PID);
			PID_Control2(DT_35_Len.spi2, expect_len, &Two_Three_PID);
			steeringWheelArray[0].expextVelocity = One_Four_PID.pid_out;
			steeringWheelArray[3].expextVelocity = One_Four_PID.pid_out;
			
			steeringWheelArray[1].expextVelocity = Two_Three_PID.pid_out;
			steeringWheelArray[2].expextVelocity = Two_Three_PID.pid_out;
		}
		
		if(Mode == AUTO)
		{
			
		}
		
		vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(20));
  }
}

void CAN_Laser_ReceiveHandler(LASER_SEND_Typedef *laser_data, uint8_t *buf) {
	laser_data->spi1 = (uint16_t)(buf[0] | (buf[1] << 8));
	laser_data->spi2 = (uint16_t)(buf[2] | (buf[3] << 8));
	laser_data->spi3 = (uint16_t)(buf[4] | (buf[5] << 8));
	laser_data->adc = (uint16_t)(buf[6] | (buf[7] << 8));
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
	if (huart->Instance == UART5)
	{
		HAL_UART_DMAStop(&huart5);
		Comm_UART_IRQ_Handle(g_comm_handle, &huart5, usart5_dma_buff,size);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart5, usart5_dma_buff,sizeof(usart5_dma_buff));
   	__HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == UART5)
	{
		HAL_UART_DMAStop(huart);
		// 重置HAL状态
		huart->ErrorCode = HAL_UART_ERROR_NONE;
		huart->RxState = HAL_UART_STATE_READY;
		huart->gState = HAL_UART_STATE_READY;
		
		// 然后清除错误标志 - 按照STM32F4参考手册要求的顺序
		uint32_t isrflags = READ_REG(huart->Instance->SR);
		
		// 按顺序处理各种错误标志，必须先读SR再读DR来清除错误
		if (isrflags & (USART_SR_ORE | USART_SR_NE | USART_SR_FE)) 
		{
				// 对于ORE、NE、FE错误，需要先读SR再读DR
				volatile uint32_t temp_sr = READ_REG(huart->Instance->SR);
				volatile uint32_t temp_dr = READ_REG(huart->Instance->DR); // 这个读取会清除ORE、NE、FE        
		}
		
		if (isrflags & USART_SR_PE)
		{
				volatile uint32_t temp_sr = READ_REG(huart->Instance->SR);
		}
		Comm_UART_IRQ_Handle(g_comm_handle, &huart5, usart5_dma_buff, 0);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart5, usart5_dma_buff,sizeof(usart5_dma_buff));
		__HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
	}
}

//中断
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	uint8_t Recv[8] = {0};
	uint32_t ID = CAN_Receive_DataFrame(hcan, Recv);
	if (hcan->Instance == CAN2)
  {
    if (ID == 0x610)
    {
      CAN_Laser_ReceiveHandler(&DT_35_Len, Recv);
    }
  }
}
