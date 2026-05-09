#ifndef _NAVIGATION__H_
#define _NAVIGATION__H_

#include <stdint.h>
#include <stdbool.h>

// ============================================
// Pure Pursuit 控制器结构体
// ============================================
typedef struct {
    // ====== 输入：当前位置（从传感器获取）======
    float current_x;       // 当前X坐标（米）
    float current_y;       // 当前Y坐标（米）
    float current_theta;   // 当前航向角（度）
    
    // ====== 输入：目标位置 ======
    float target_x;        // 目标X坐标（米）
    float target_y;        // 目标Y坐标（米）
    float target_theta;    // 目标航向角（度）
    
    // ====== 控制参数 ======
    float max_velocity;       // 最大速度（米/秒）
    float k_lateral;          // 横向误差增益
    
    // ====== 容差参数 ======
    float position_tolerance;  // 位置容差（米）
    
    // ====== 输出：控制量（车体坐标系）======
    float vx;              // X方向速度（米/秒）
    float vy;              // Y方向速度（米/秒）
    
    // ====== 运行状态 ======
    bool is_running;          // 是否正在运行
} PurePursuitController;

void PurePursuit_Init(PurePursuitController *controller);
void PurePursuit_SetTarget(PurePursuitController *controller, float target_x, float target_y);
void PurePursuit_Update(PurePursuitController* controller);

#endif