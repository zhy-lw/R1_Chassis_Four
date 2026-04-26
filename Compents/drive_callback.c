#include "drive_callback.h"
#include "semphr.h"

void SetWheelTarget_Callback(Wheel_t *_this, float rad, float velocity, float force)
{
    SteeringWheel *steeringwheel = (SteeringWheel *)_this->user_data;
    steeringwheel->expectDirection = RAD2ANGLE(rad);
    steeringwheel->expextVelocity = velocity;
}