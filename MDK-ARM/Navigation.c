#include "Navigation.h"
#include <math.h>
#include "Task_Init.h"
#include "arm_math.h"

void PurePursuit_Init(PurePursuitController *controller)
{
    // 初始化控制器参数
    controller->max_velocity = 1.0f;//最大速度
    controller->k_lateral = 2.0f;//横向误差增益

    // 容差参数
    controller->position_tolerance = 0.03f;//位置容差

    // 初始化输出控制量
    controller->vx = 0.0f;
    controller->vy = 0.0f;

    controller->is_running = false;
}

void PurePursuit_SetTarget(PurePursuitController *controller, float target_x, float target_y, float target_theta)
{
    controller->target_x = target_x;
    controller->target_y = target_y;
    controller->target_theta = target_theta;
    controller->is_running = true;
}

float NormalizeAngle(float angle) {
    while (angle > 180.0f) {
        angle -= 360.0f;
    }
    while (angle < -180.0f) {
        angle += 360.0f;
    }
    return angle;
}

float CalculateDistance(float x1, float y1, float x2, float y2) {
    float dx = x2 - x1;
    float dy = y2 - y1;
    return sqrtf(dx * dx + dy * dy);
}

float CalculateLateralError(float current_x, float current_y,
                           float target_x, float target_y,
                           float current_theta) {
    float dx = target_x - current_x;
    float dy = target_y - current_y;
    float theta_rad = current_theta * PI / 180.0f;
    return -arm_sin_f32(theta_rad) * dx + arm_cos_f32(theta_rad) * dy;
}

void PurePursuit_Update(PurePursuitController* controller)
{
    float dx = controller->target_x - controller->current_x;
    float dy = controller->target_y - controller->current_y;
    float distance = CalculateDistance(controller->current_x, controller->current_y,
                                       controller->target_x, controller->target_y);
    if (distance < controller->position_tolerance) {
        controller->vx = 0.0f;
        controller->vy = 0.0f;
        controller->is_running = false;
        return;
    }

    // 指向目标方向
    float target_angle_rad = arm_atan2_f32(dy, dx);
    float vx_raw = arm_cos_f32(target_angle_rad) * controller->max_velocity;
    float vy_raw = arm_sin_f32(target_angle_rad) * controller->max_velocity;

    // 横向误差修正
    float lateral_error = CalculateLateralError(
        controller->current_x, controller->current_y,
        controller->target_x, controller->target_y,
        controller->current_theta);
    vx_raw += arm_sin_f32(target_angle_rad) * controller->k_lateral * lateral_error;
    vy_raw += arm_cos_f32(target_angle_rad) * controller->k_lateral * lateral_error;

    // 速度限幅
    float v_total = sqrtf(vx_raw * vx_raw + vy_raw * vy_raw);
    if (v_total > controller->max_velocity) {
        float scale = controller->max_velocity / v_total;
        controller->vx = vx_raw * scale;
        controller->vy = vy_raw * scale;
    } else {
        controller->vx = vx_raw;
        controller->vy = vy_raw;
    }

}
