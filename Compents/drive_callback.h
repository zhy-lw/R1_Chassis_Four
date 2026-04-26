#ifndef DRIVE_CALLBACK_H
#define DRIVE_CALLBACK_H

#include "ForceChassis.h"
#include "FreeRTOS.h"
#include "Task.h"
#include "pid_old.h"
#include "motor.h"
#include "Odrive.h"
#include "vesc.h"
#include "encoder.h"

#define PI 3.14159265358979f
#define ANGLE2RAD(x) (x) * PI / 180.0f
#define RAD2ANGLE(x) (x) * 180.0f / PI
#define KV 170

typedef struct
{
  float expectDirection; //期望方向角度（°）
  float expextVelocity;
  float expextForce;
} SteeringWheel;

/*回调函数*/
void SetWheelTarget_Callback(Wheel_t *_this, float rad, float velocity, float force);

#endif
