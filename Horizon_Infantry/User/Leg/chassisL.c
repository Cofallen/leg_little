#include "chassisL.h"
#include "MY_Define.h"
#include "DM_Motor.h"
#include "controller.h"
#include "All_Init.h"
#include "vmc.h"
#include "get_K.h"
#include "pid_temp.h"



void ChassisL_Init(void)
{
    motor_mode(&hcan1, LEG_LF+1, 0x000, 0xfc);
    osDelay(1);
    motor_mode(&hcan1, LEG_LB+1, 0x000, 0xfc);
    osDelay(1);
    ALL_MOTOR.left_wheel.DATA.Angle_Init = ALL_MOTOR.left_wheel.DATA.Angle_Infinite;
}

void ChassisL_UpdateState(Leg_Typedef *object, MOTOR_Typedef *motor, IMU_Data_t *imu, float dt)
{
    // 更新状态
    object->stateSpace.theta = (PI / 2.0f - object->vmc_calc.phi0[POS] + imu->pitch /  57.3f);
    object->stateSpace.dtheta = Discreteness_Diff(&object->Discreteness.Theta, object->stateSpace.theta, dt);
    object->stateSpace.phi = -imu->pitch / 57.3f;
    object->stateSpace.dphi = Discreteness_Diff(&object->Discreteness.Phi, object->stateSpace.phi, dt);

    object->stateSpace.ddtheta = Discreteness_Diff(&object->Discreteness.dTheta, object->stateSpace.dtheta, dt);
}

// 用于更新两腿之间的相对状态
void Chassis_UpdateStateS(Leg_Typedef *Leg_l, Leg_Typedef *Leg_r, MOTOR_Typedef *motor, float dt)
{
    float s = 0.0f, dot_s_b = 0.0f, dot_s = 0.0f;
    float theta_wl = 0.0f, theta_wr = 0.0f;
    float dtheta_wl = 0.0f, dtheta_wr = 0.0f;

    theta_wl = -(float)(motor->left_wheel.DATA.Angle_Infinite - motor->left_wheel.DATA.Angle_Init) / 8192.0f * 360.0f / 57.3f / REDUCTION_RATIO;
    theta_wr =  (float)(motor->right_wheel.DATA.Angle_Infinite - motor->right_wheel.DATA.Angle_Init) / 8192.0f * 360.0f / 57.3f / REDUCTION_RATIO;
    dtheta_wl = Discreteness_Diff(&Leg_l->Discreteness.Theta_w, theta_wl, dt);
    dtheta_wr = Discreteness_Diff(&Leg_r->Discreteness.Theta_w, theta_wr, dt);

    s = RADIUS_WHEEL * (theta_wl + theta_wr) / 2.0f;
    dot_s_b = RADIUS_WHEEL * (dtheta_wl + dtheta_wr) / 2.0f;

    dot_s = dot_s_b + 0.5f * (Leg_l->vmc_calc.L0[POS] * Leg_r->stateSpace.dtheta * arm_cos_f32(Leg_l->stateSpace.theta) + Leg_r->vmc_calc.L0[POS] * Leg_l->stateSpace.dtheta * arm_cos_f32(Leg_r->stateSpace.theta)) \
                      + 0.5f * (Leg_l->vmc_calc.L0[VEL] * arm_sin_f32(Leg_l->stateSpace.theta) + Leg_r->vmc_calc.L0[VEL] * arm_sin_f32(Leg_r->stateSpace.theta));

    Leg_l->stateSpace.dot_s = dot_s;
    Leg_r->stateSpace.dot_s = dot_s;
    Leg_l->stateSpace.s     = Discreteness_Sum(&Leg_l->Discreteness.dS, Leg_l->stateSpace.dot_s, dt);
    Leg_r->stateSpace.s     = Discreteness_Sum(&Leg_r->Discreteness.dS, Leg_r->stateSpace.dot_s, dt);

    Leg_l->LQR.delta = Leg_r->stateSpace.theta + Leg_l->stateSpace.theta;
}


void ChassisL_Control(Leg_Typedef *object, DBUS_Typedef *dbus, IMU_Data_t *imu)
{
    memcpy(object->LQR.K, ChassisL_LQR_K, sizeof(float) * 12);
    
    // 目标值获取应加上滤波 重写一个函数
    object->target.theta = 0.0f;
    // object->target.theta = -0.008f;
    object->target.dtheta = 0.0f;
    object->target.s = 0.0f;
    object->target.dot_s = (float)dbus->Remote.CH1_int16 / 600.0f;
    object->target.phi = 0.0f;
    object->target.dphi = 0.0f;

    object->LQR.T_w = (ChassisL_LQR_K[0] * (object->stateSpace.theta - object->target.theta) +
                     ChassisL_LQR_K[1] * (object->stateSpace.dtheta - object->target.dtheta) +
                     ChassisL_LQR_K[2] * (object->stateSpace.s - object->target.s) +
                     ChassisL_LQR_K[3] * (object->stateSpace.dot_s - object->target.dot_s) +
                     ChassisL_LQR_K[4] * (object->stateSpace.phi - object->target.phi) +
                     ChassisL_LQR_K[5] * (object->stateSpace.dphi - object->target.dphi));

    object->LQR.T_p = (ChassisL_LQR_K[6] * (object->stateSpace.theta - object->target.theta) +
                      ChassisL_LQR_K[7] * (object->stateSpace.dtheta - object->target.dtheta) +
                      ChassisL_LQR_K[8] * (object->stateSpace.s - object->target.s) +
                      ChassisL_LQR_K[9] * (object->stateSpace.dot_s - object->target.dot_s) +
                      ChassisL_LQR_K[10] * (object->stateSpace.phi - object->target.phi) +
                      ChassisL_LQR_K[11] * (object->stateSpace.dphi - object->target.dphi));

    PID_calc(&object->pid.F0_l, object->target.l0, object->vmc_calc.L0[POS]);
    object->LQR.dF_0 = object->pid.F0_l.out;

    PID_calc(&object->pid.Roll, object->target.roll, imu->roll / 57.3f);
    object->LQR.dF_roll = object->pid.Roll.out;

    PID_calc(&object->pid.Delta, object->target.d2theta, object->LQR.delta);
    object->LQR.dF_delta = object->pid.Delta.out;

    object->LQR.F_0 = -(object->LQR.dF_0 - object->LQR.dF_roll);
    object->LQR.F_0 = (MASS_BODY / 2.0f * 9.81f / arm_cos_f32(object->stateSpace.theta) - object->LQR.dF_0 - object->LQR.dF_roll);
    // object->LQR.F_0 = 0;
    // pid修正
    object->LQR.T_p = object->LQR.T_p + object->LQR.dF_delta;
    // object->LQR.T_w = object->LQR.T_w - object->LQR.dF_yaw;


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


void Chassis_SendTorque()
{
    RUI_V_CONTAL.DWT_TIME.TIM7_Dtime = DWT_GetDeltaT(&RUI_V_CONTAL.DWT_TIME.TIM7_DWT_Count);
    static uint8_t temp = 1;
    uint8_t count = 0;
    if (temp == 1){
      mit_ctrl(&hcan1, 0x01, 0,0,0,0, -Leg_l.LQR.torque_setT[0]);
      mit_ctrl(&hcan1, 0x03, 0,0,0,0, -Leg_l.LQR.torque_setT[1]);
      // mit_ctrl(&hcan1, 0x01, 0, 0, 0, 0, 0);
      // mit_ctrl(&hcan1, 0x03, 0, 0, 0, 0, 0);
      // DJI_Torque_Control(&hcan2, 0x200, 0.0f, 0.0f, Leg_l.LQR.torque_setW, 0.0f);
      // DJI_Torque_Control(&hcan2, 0x200, Leg_r.LQR.torque_setW, 0.0f, 0.0f, 0.0f);
      // DJI_Torque_Control(&hcan2, 0x200, Leg_r.LQR.torque_setW, 0.0f, Leg_l.LQR.torque_setW, 0.0f);
      temp = -temp;
    }
    else{
      mit_ctrl(&hcan1, 0x02, 0,0,0,0, Leg_r.LQR.torque_setT[0]);
      mit_ctrl(&hcan1, 0x04, 0,0,0,0, Leg_r.LQR.torque_setT[1]);
      // mit_ctrl(&hcan1, 0x02, 0,0,0,0, 0);
      // mit_ctrl(&hcan1, 0x04, 0,0,0,0, 0);
      temp = -temp;
    }
}