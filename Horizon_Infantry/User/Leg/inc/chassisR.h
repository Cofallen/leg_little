#ifndef __CHASSISR_H
#define __CHASSISR_H

#include "vmc.h"

void ChassisR_Init(void);
void ChassisR_UpdateState(Leg_Typedef *object, MOTOR_Typedef *motor, IMU_Data_t *imu, float dt);

#endif // !__CHASSISR_H