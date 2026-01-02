#include "chassisR.h"
#include "MY_Define.h"
#include "DM_Motor.h"
#include "All_Init.h"
#include "vmc.h"
#include "get_K.h"

void ChassisR_Init(void)
{
    motor_mode(&hcan1, LEG_RF+1, 0x000, 0xfc);
    osDelay(1);
    motor_mode(&hcan1, LEG_RB+1, 0x000, 0xfc);
    osDelay(1);
    ALL_MOTOR.right_wheel.DATA.Angle_Init = ALL_MOTOR.right_wheel.DATA.Angle_Infinite;
}

void ChassisR_UpdateState(Leg_Typedef *object, MOTOR_Typedef *motor, IMU_Data_t *imu, float dt)
{
    // 更新状态
    object->stateSpace.theta = (-PI / 2.0f + object->vmc_calc.phi0[POS] + imu->pitch /  57.3f);
    object->stateSpace.dtheta = Discreteness_Diff(&object->Discreteness.Theta, object->stateSpace.theta, dt);
    object->stateSpace.phi = imu->pitch / 57.3f;
    object->stateSpace.dphi = Discreteness_Diff(&object->Discreteness.Phi, object->stateSpace.phi, dt);

    object->stateSpace.ddtheta = Discreteness_Diff(&object->Discreteness.dTheta, object->stateSpace.dtheta, dt);
}



void ChassisR_Control(Leg_Typedef *object, DBUS_Typedef *dbus, IMU_Data_t *imu, float dt)
{
    memcpy(object->LQR.K, ChassisR_LQR_K, sizeof(float) * 12);
    
    // 目标值获取应加上滤波 重写一个函数
    object->target.theta = 0.0f;
    // object->target.theta = -0.008f;
    object->target.dtheta = 0.0f;
    object->target.dot_s = (float)dbus->Remote.CH1_int16 / 600.0f;
    object->target.s = Discreteness_Sum(&object->Discreteness.target_s, object->target.dot_s, dt);
    object->target.phi = 0.0f;
    object->target.dphi = 0.0f;

    object->LQR.T_w = (ChassisR_LQR_K[0] * (object->stateSpace.theta - object->target.theta) +
                     ChassisR_LQR_K[1] * (object->stateSpace.dtheta - object->target.dtheta) +
                     ChassisR_LQR_K[2] * (object->stateSpace.s - object->target.s) +
                     ChassisR_LQR_K[3] * (object->stateSpace.dot_s - object->target.dot_s) +
                     ChassisR_LQR_K[4] * (object->stateSpace.phi - object->target.phi) +
                     ChassisR_LQR_K[5] * (object->stateSpace.dphi - object->target.dphi));

    object->LQR.T_p = (ChassisR_LQR_K[6] * (object->stateSpace.theta - object->target.theta) +
                      ChassisR_LQR_K[7] * (object->stateSpace.dtheta - object->target.dtheta) +
                      ChassisR_LQR_K[8] * (object->stateSpace.s - object->target.s) +
                      ChassisR_LQR_K[9] * (object->stateSpace.dot_s - object->target.dot_s) +
                      ChassisR_LQR_K[10] * (object->stateSpace.phi - object->target.phi) +
                      ChassisR_LQR_K[11] * (object->stateSpace.dphi - object->target.dphi));

    PID_calc(&object->pid.F0_l, object->vmc_calc.L0[POS], object->target.l0);
    object->LQR.dF_0 = object->pid.F0_l.out;

    PID_calc(&object->pid.Roll, imu->roll / 57.3f, object->target.roll);
    object->LQR.dF_roll = object->pid.Roll.out;

    PID_calc(&object->pid.Delta, object->LQR.delta, object->target.d2theta);
    object->LQR.dF_delta = object->pid.Delta.out;

    object->LQR.F_0 = (MASS_BODY / 2.0f * 9.81f / arm_cos_f32(object->stateSpace.theta) + object->LQR.dF_0 - object->LQR.dF_roll);
    // object->LQR.F_0 = object->LQR.dF_0 - object->LQR.dF_roll;

    // pid修正
    object->LQR.T_p = object->LQR.T_p - object->LQR.dF_delta;
    object->LQR.T_w = object->LQR.T_w - object->LQR.dF_yaw;


    object->LQR.torque_setT[0] = object->vmc_calc.JRM[0][0] * object->LQR.F_0 + \
                                 object->vmc_calc.JRM[0][1] * object->LQR.T_p;
    object->LQR.torque_setT[1] = object->vmc_calc.JRM[1][0] * object->LQR.F_0 + \
                                 object->vmc_calc.JRM[1][1] * object->LQR.T_p;
    object->LQR.torque_setW  = object->LQR.T_w;

    // 限幅
    (object->LQR.torque_setT[0] > MAX_TORQUE_LEG_T) ? (object->LQR.torque_setT[0] = MAX_TORQUE_LEG_T) : (object->LQR.torque_setT[0] < MIN_TORQUE_LEG_T) ? (object->LQR.torque_setT[0] = MIN_TORQUE_LEG_T) : 0;
    (object->LQR.torque_setT[1] > MAX_TORQUE_LEG_T) ? (object->LQR.torque_setT[1] = MAX_TORQUE_LEG_T) : (object->LQR.torque_setT[1] < MIN_TORQUE_LEG_T) ? (object->LQR.torque_setT[1] = MIN_TORQUE_LEG_T) : 0;
    (object->LQR.torque_setW > MAX_TORQUE_LEG_W) ? (object->LQR.torque_setW = MAX_TORQUE_LEG_W) : (object->LQR.torque_setW < MIN_TORQUE_LEG_W) ? (object->LQR.torque_setW = MIN_TORQUE_LEG_W) : 0;
}