#include "VL53_100.h"

/**
 * @brief 解析传感器数据
 * @param 获取数据状态
 * @param 获取距离(mm)
 * @return 
 */
int VL53_100_DataProcess(uint8_t *buffer,VL53_Data_t *para)
{
	int i=0,j=0,d=0;
	
  /*解析数据状态*/
	if(buffer[8]== ',')                          //state:0 -> 5
	{
    if (buffer[6] == '0') {
     para->state = VL53_STATE_RANGE_VALID;
    } else if (buffer[6] == '1') {
        para->state = VL53_STATE_SIGMA_FAIL;
			  return 0;
    } else if (buffer[6] == '2') {
        para->state = VL53_STATE_SIGNAL_FAIL;
				return 0;
    } else if (buffer[6] == '3') {
        para->state = VL53_STATE_MIN_RANGE;
			  return 0;
    } else if (buffer[6] == '4') {
        para->state =VL53_STATE_PHASE_FAIL;
			  para->distance_mm = 1200;
				return 0;
    } else if (buffer[6] == '5') {
        para->state = VL53_STATE_HW_FAIL;
				return 0;
    } 
	}
	else                                         //无数据更新(state:255)
  {
       para->state = VL53_STATE_NO_UPDATE;
		   return 0;
  }
	
 /*解析距离*/
 for(i=0;buffer[i]!='\n';i++);                 //直接定位到第一行末尾
 for(j=10;j!='m';j++)                           //检测整个data数字域
 {
		if(buffer[j]>='0'&&buffer[j]<='9')
			d=d*10+buffer[j]-'0';
 }
 para->distance_mm = d;
	
	return 1;
}


