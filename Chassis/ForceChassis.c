#include "ForceChassis.h"
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "matrix.h"

float velocity[8] = {0};

void ChassisCalculateProcess(void *param)
{
    Chassis_t *chassis = (Chassis_t *)param;
    int wheel_num=chassis->wheel_num;

    float force[8];
    float cur_velocity[8];
    float robot_force[3];
    float robot_cur_vel[3];

    //底盘当前速度向量
    arm_matrix_instance_f32 wheel_vel_mat, wheel_cur_vel_mat,robot_vel_mat, wheel_force_mat, robot_force_mat, robot_acc_mat,robot_cur_vel_mat;
    arm_mat_init_f32(&wheel_force_mat, wheel_num*2, 1, force);    //轮子力矩向量
    arm_mat_init_f32(&wheel_vel_mat, wheel_num*2, 1, velocity);   //轮子速度向量
    arm_mat_init_f32(&wheel_cur_vel_mat, wheel_num*2, 1, cur_velocity);   //轮子当前速度向量

    arm_mat_init_f32(&robot_force_mat, 3, 1, robot_force);  //底盘广义力向量
    arm_mat_init_f32(&robot_vel_mat, 3, 1, (float *)(&(chassis->exp_vel)));   //底盘广义速度向量
    arm_mat_init_f32(&robot_acc_mat, 3, 1, (float *)(&(chassis->exp_acc)));   //底盘广义加速度向量

    arm_mat_init_f32(&robot_cur_vel_mat,3,1,robot_cur_vel);

    TickType_t last_wake_time = xTaskGetTickCount();
    while (1)
    {
        // 逆解部分:
        // TODO:底盘期望速度->轮子期望速度
        arm_mat_mult_f32(&chassis->vel_A_mat, &robot_vel_mat, &wheel_vel_mat);

        for (int i = 0; i < wheel_num; i++)     //设置期望到轮子上
        {
            float torque_projection,exp_dir=chassis->wheel[i]->last_rad,exp_vel;

            exp_vel = sqrtf(velocity[2*i]*velocity[2*i]+velocity[2*i+1]*velocity[2*i+1]);
            if (exp_vel > chassis->dead_zone) // 防止奇点
            {
                exp_dir = atan2f(velocity[2*i+1], velocity[2*i]);
                torque_projection = (velocity[2*i] * force[2*i] + velocity[2*i+1] * force[2*i+1]) / exp_vel;
            }
            else
                torque_projection = 0.0f;

            chassis->wheel[i]->set_target_cb(chassis->wheel[i], exp_dir, exp_vel, torque_projection);
						chassis->wheel[i]->last_rad=exp_dir;
        }

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(chassis->update_dt_ms));
    }
}

TaskHandle_t task_handle;

uint32_t ChassisInit(Chassis_t *chassis, Wheel_t *wheel,uint32_t wheel_num, Vector2D barycenter, float mass, float I, float dead_zone, uint32_t update_dt, uint32_t task_stack_size, uint32_t task_priority) // 初始化数学参数并创建任务
{
    if(wheel_num<2)
        return pdFAIL;
    chassis->barycenter = barycenter;
    chassis->mass = mass;
    chassis->I = I;
    chassis->dead_zone = dead_zone;
    chassis->update_dt_ms = update_dt;
    chassis->wheel_num=wheel_num;

    //数学映射矩阵初始化
    arm_mat_init_f32(&chassis->Mq_mat, 3, 3, (float*)chassis->Mq_data); // 构建惯性矩阵
    chassis->Mq_data[1][1] = chassis->Mq_data[0][0] = chassis->mass;
    chassis->Mq_data[0][1] = chassis->Mq_data[1][0] = 0.0f;
    chassis->Mq_data[0][2] = chassis->Mq_data[2][0] = -chassis->mass * chassis->barycenter.y;
    chassis->Mq_data[1][2] = chassis->Mq_data[2][1] = chassis->mass * chassis->barycenter.x;
    chassis->Mq_data[2][2] = chassis->I + chassis->mass * chassis->barycenter.x * chassis->barycenter.x +chassis->mass * chassis->barycenter.y * chassis->barycenter.y;
    
    arm_mat_init_f32(&chassis->vel_A_mat,wheel_num*2,3,(float*)&chassis->A_vel_data);//构建运动学映射矩阵
    for(int i=0;i<wheel_num;i++)
    {
        Vector3D pos=chassis->wheel[i]->pos;

        float c=cosf(pos.z);
        float s=sinf(pos.z);
        chassis->A_vel_data[i*2][0]=c;
        chassis->A_vel_data[i*2][1]=s;
        chassis->A_vel_data[i*2][2]=-pos.y*c+pos.x*s;
        chassis->A_vel_data[i*2+1][0]=-s;
        chassis->A_vel_data[i*2+1][1]=c;
        chassis->A_vel_data[i*2+1][2]=pos.y*s+pos.x*c;
    }

    arm_mat_init_f32(&chassis->force_A_mat,3,wheel_num*2,(float*)&chassis->force_A_data);
    arm_mat_trans_f32(&chassis->vel_A_mat,&chassis->force_A_mat);       //计算动力学映射矩阵

    return xTaskCreate(ChassisCalculateProcess, "chassis_task", task_stack_size, chassis, task_priority, &task_handle);
}
