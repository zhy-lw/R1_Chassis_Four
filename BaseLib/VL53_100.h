#ifndef __VL53_100_H
#define __VL53_100_H

#include "stdint.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"


/*数据状态标志枚举*/
typedef enum {
    VL53_STATE_RANGE_VALID = 0,    // 测量范围内
    VL53_STATE_SIGMA_FAIL  = 1,    // Sigma错误
    VL53_STATE_SIGNAL_FAIL = 2,    // 信号错误
    VL53_STATE_MIN_RANGE   = 3,    // 小于最小测量范围
    VL53_STATE_PHASE_FAIL  = 4,    // 超出测量范围
    VL53_STATE_HW_FAIL     = 5,    // 硬件错误
    VL53_STATE_NO_UPDATE   = 255   // 无数据更新
}VL53_State_t;

/*解析结果结构体*/
typedef struct {
    VL53_State_t state;              // 当前状态
    uint32_t distance_mm ;         // 距离数据(mm)
}VL53_Data_t;

int VL53_100_DataProcess(uint8_t *buffer,VL53_Data_t *para);


#endif
