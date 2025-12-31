#ifndef __CHASSISL_H
#define __CHASSISL_H

#include "vmc.h"

void ChassisL_Init(void);
void ChassisL_UpdateState(Leg_Typedef *object, MOTOR_Typedef *motor, IMU_Data_t *imu, float dt);
float Chassis_UpdateStateS(Leg_Typedef *Leg_l, Leg_Typedef *Leg_r, MOTOR_Typedef *motor, float dt);


#endif // !__CHASSISL_H