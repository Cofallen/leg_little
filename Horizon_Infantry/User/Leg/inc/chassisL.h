#ifndef __CHASSISL_H
#define __CHASSISL_H

#include "vmc.h"
#include "RUI_DBUS.h"

void ChassisL_Init(MOTOR_Typedef *motor, Leg_Typedef *object);
void ChassisL_UpdateState(Leg_Typedef *object, MOTOR_Typedef *motor, IMU_Data_t *imu, float dt);
void Chassis_UpdateStateS(Leg_Typedef *Leg_l, Leg_Typedef *Leg_r, MOTOR_Typedef *motor, float dt);
void ChassisL_Control(Leg_Typedef *object, DBUS_Typedef *dbus, IMU_Data_t *imu, float dt);
void Chassis_SendTorque();
void Chassis_GetStatus(Leg_Typedef *left, Leg_Typedef *right);
void Chassis_StateHandle(Leg_Typedef *left, Leg_Typedef *right);
void Chassis_Rotate(MOTOR_Typedef *motor, Leg_Typedef *left, Leg_Typedef *right);
void Chassis_GetTorque(MOTOR_Typedef *motor, Leg_Typedef *left, Leg_Typedef *right, DBUS_Typedef *dbus);

#endif // !__CHASSISL_H