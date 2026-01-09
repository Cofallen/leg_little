#include "chassisL.h"
#include "MY_Define.h"
#include "DM_Motor.h"
#include "controller.h"
#include "All_Init.h"
#include "vmc.h"
#include "get_K.h"
#include "pid_temp.h"

float PID_S_LF[3] = {5.0f, 0.0f, 0.0f};
float PID_P_LF[3] = {1.0f, 0.0f, 0.0f};
float PID_S_LB[3] = {5.0f, 0.0f, 0.0f};
float PID_P_LB[3] = {1.0f, 0.0f, 0.0f};

void ChassisL_Init(MOTOR_Typedef *motor, Leg_Typedef *object)
{
    motor_mode(&hcan1, LEG_LF+1, 0x000, 0xfc);
    osDelay(1);
    motor_mode(&hcan1, LEG_LB+1, 0x000, 0xfc);
    osDelay(1);
    ALL_MOTOR.left_wheel.DATA.Angle_Init = ALL_MOTOR.left_wheel.DATA.Angle_Infinite;
    PID_Init(&motor->left_front.PID_P, 1.0f, 0.1f, PID_P_LF,
              2000.0f, 1000.0f, 0.7f, 0.7f, 2, 
              Integral_Limit|OutputFilter|ErrorHandle|
              Trapezoid_Intergral|ChangingIntegrationRate|
              Derivative_On_Measurement|DerivativeFilter);
    PID_Init(&motor->left_front.PID_S, 1.5f, 0.1f, PID_S_LF,
              2000.0f, 1000.0f, 0.7f, 0.7f, 2, 
              Integral_Limit|OutputFilter|ErrorHandle|
              Trapezoid_Intergral|ChangingIntegrationRate|
              Derivative_On_Measurement|DerivativeFilter);
    PID_Init(&motor->left_back.PID_P, 1.0f, 0.1f, PID_P_LB,
              2000.0f, 1000.0f, 0.7f, 0.7f, 2, 
              Integral_Limit|OutputFilter|ErrorHandle|
              Trapezoid_Intergral|ChangingIntegrationRate|
              Derivative_On_Measurement|DerivativeFilter);
    PID_Init(&motor->left_back.PID_S, 1.5f, 0.1f, PID_S_LB,
              2000.0f, 1000.0f, 0.7f, 0.7f, 2, 
              Integral_Limit|OutputFilter|ErrorHandle|
              Trapezoid_Intergral|ChangingIntegrationRate|
              Derivative_On_Measurement|DerivativeFilter);
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

    Leg_l->LQR.delta = Leg_r->stateSpace.theta - Leg_l->stateSpace.theta;
    Leg_r->LQR.delta = Leg_r->stateSpace.theta - Leg_l->stateSpace.theta;
}


void ChassisL_Control(Leg_Typedef *object, DBUS_Typedef *dbus, IMU_Data_t *imu, float dt)
{   
    // 目标值获取应加上滤波 重写一个函数
    object->target.theta = (float)dbus->Remote.CH1_int16 / 660.0f * 1.0f;
    // object->target.theta = -0.008f;
    object->target.dtheta = 0.0f;
    object->target.dot_s = (float)dbus->Remote.CH1_int16 / 660.0f * 1.5f;
    object->target.s = Discreteness_Sum(&object->Discreteness.target_s, object->target.dot_s, dt);
    // object->target.phi = 0.0f;
    // object->target.dphi = 0.0f;
    object->target.yaw -= (float)dbus->Remote.CH2_int16 / 660000.0f * 4.0f;
    object->target.l0 += (float)dbus->Remote.CH3_int16 / 660000.0f; 
    (object->target.l0 > MAX_LEG_LENGTH) ? (object->target.l0 = MAX_LEG_LENGTH) : (object->target.l0 < MIN_LEG_LENGTH) ? (object->target.l0 = MIN_LEG_LENGTH) : 0;


    object->LQR.T_w = (object->LQR.K[0] * (object->stateSpace.theta - object->target.theta) +
                     object->LQR.K[1] * (object->stateSpace.dtheta - object->target.dtheta) +
                     object->LQR.K[2] * (object->stateSpace.s - object->target.s) +
                     object->LQR.K[3] * (object->stateSpace.dot_s - object->target.dot_s) +
                     object->LQR.K[4] * (object->stateSpace.phi - object->target.phi) +
                     object->LQR.K[5] * (object->stateSpace.dphi - object->target.dphi));

    object->LQR.T_p = (object->LQR.K[6] * (object->stateSpace.theta - object->target.theta) +
                      object->LQR.K[7] * (object->stateSpace.dtheta - object->target.dtheta) +
                      object->LQR.K[8] * (object->stateSpace.s - object->target.s) +
                      object->LQR.K[9] * (object->stateSpace.dot_s - object->target.dot_s) +
                      object->LQR.K[10] * (object->stateSpace.phi - object->target.phi) +
                      object->LQR.K[11] * (object->stateSpace.dphi - object->target.dphi));

    PID_calc(&object->pid.F0_l, object->vmc_calc.L0[POS], object->target.l0);
    object->LQR.dF_0 = object->pid.F0_l.out;

    PID_calc(&object->pid.Roll, imu->roll / 57.3f, object->target.roll);
    object->LQR.dF_roll = object->pid.Roll.out;

    PID_calc(&object->pid.Delta, object->LQR.delta, object->target.d2theta);
    object->LQR.dF_delta = object->pid.Delta.out;

    PID_calc(&object->pid.Yaw, imu->YawTotalAngle / 57.3f, object->target.yaw);
    object->LQR.dF_yaw = object->pid.Yaw.out;
    // object->LQR.F_0 = -(object->LQR.dF_0 - object->LQR.dF_roll);
    object->LQR.F_0 = (MASS_BODY / 2.0f * 9.81f / arm_cos_f32(object->stateSpace.theta) + object->LQR.dF_0 + object->LQR.dF_roll);
    // object->LQR.F_0 = 0;
    // pid修正
    object->LQR.T_p = object->LQR.T_p + object->LQR.dF_delta;
    object->LQR.T_w = object->LQR.T_w + object->LQR.dF_yaw;

    object->LQR.torque_setT[0] = object->vmc_calc.JRM[0][0] * object->LQR.F_0 + \
                                 object->vmc_calc.JRM[0][1] * object->LQR.T_p;
    object->LQR.torque_setT[1] = object->vmc_calc.JRM[1][0] * object->LQR.F_0 + \
                                 object->vmc_calc.JRM[1][1] * object->LQR.T_p;
    object->LQR.torque_setW  = object->LQR.T_w;

    // 限幅
    // (object->LQR.torque_setT[0] > object->limit.T_max) ? (object->LQR.torque_setT[0] = object->limit.T_max) : (object->LQR.torque_setT[0] < -object->limit.T_max) ? (object->LQR.torque_setT[0] = -object->limit.T_max) : 0;
    // (object->LQR.torque_setT[1] > object->limit.T_max) ? (object->LQR.torque_setT[1] = object->limit.T_max) : (object->LQR.torque_setT[1] < -object->limit.T_max) ? (object->LQR.torque_setT[1] = -object->limit.T_max) : 0;
    // (object->LQR.torque_setW > object->limit.W_max) ? (object->LQR.torque_setW = -object->limit.W_max) : (object->LQR.torque_setT[0] < -object->limit.W_max) ? (object->LQR.torque_setW = -object->limit.W_max) : 0;
    if (dbus->Remote.S2_u8 == 1)
    {
      object->LQR.torque_setT[0] = 0.0f;
      object->LQR.torque_setT[1] = 0.0f;
      object->LQR.torque_setW = 0.0f;
    }
    (object->LQR.torque_setT[0] > MAX_TORQUE_LEG_T) ? (object->LQR.torque_setT[0] = MAX_TORQUE_LEG_T) : (object->LQR.torque_setT[0] < -MAX_TORQUE_LEG_T) ? (object->LQR.torque_setT[0] = -MAX_TORQUE_LEG_T) : 0;
    (object->LQR.torque_setT[1] > MAX_TORQUE_LEG_T) ? (object->LQR.torque_setT[1] = MAX_TORQUE_LEG_T) : (object->LQR.torque_setT[1] < -MAX_TORQUE_LEG_T) ? (object->LQR.torque_setT[1] = -MAX_TORQUE_LEG_T) : 0;
    (object->LQR.torque_setW > MAX_TORQUE_LEG_W) ? (object->LQR.torque_setW = MAX_TORQUE_LEG_W) : (object->LQR.torque_setW < -MAX_TORQUE_LEG_W) ? (object->LQR.torque_setW = -MAX_TORQUE_LEG_W) : 0;
}


void Chassis_SendTorque()
{
    RUI_V_CONTAL.DWT_TIME.TIM7_Dtime = DWT_GetDeltaT(&RUI_V_CONTAL.DWT_TIME.TIM7_DWT_Count);
    static uint8_t temp = 1;
    uint8_t count = 0;
    if (temp == 1){
      mit_ctrl(&hcan1, 0x01, 0,0,0,0, Leg_l.torque_send.T1);
      mit_ctrl(&hcan1, 0x03, 0,0,0,0, Leg_l.torque_send.T2);
      // mit_ctrl(&hcan1, 0x01, 0, 0, 0, 0, 0);
      // mit_ctrl(&hcan1, 0x03, 0, 0, 0, 0, 0);
      // DJI_Torque_Control(&hcan2, 0x200, 0.0f, 0.0f, Leg_l.LQR.torque_setW, 0.0f);
      // DJI_Torque_Control(&hcan2, 0x200, -Leg_r.LQR.torque_setW, 0.0f, 0.0f, 0.0f);
      DJI_Torque_Control(&hcan2, 0x200, Leg_r.torque_send.Tw, 0.0f, Leg_l.torque_send.Tw, 0.0f);
      temp = -temp;
    }
    else{
      mit_ctrl(&hcan1, 0x02, 0,0,0,0, Leg_r.torque_send.T1);
      mit_ctrl(&hcan1, 0x04, 0,0,0,0, Leg_r.torque_send.T2);
      // mit_ctrl(&hcan1, 0x02, 0,0,0,0, 0);
      // mit_ctrl(&hcan1, 0x04, 0,0,0,0, 0);
      temp = -temp;
    }
}

// 决定输出力矩，选择
void Chassis_GetTorque(MOTOR_Typedef *motor, Leg_Typedef *left, Leg_Typedef *right, DBUS_Typedef *dbus)
{
  left->torque_send.T1  = -left->LQR.torque_setT[0];
  left->torque_send.T2  = -left->LQR.torque_setT[1];
  left->torque_send.Tw  =  left->LQR.torque_setW;
  right->torque_send.T1 =  right->LQR.torque_setT[0];
  right->torque_send.T2 =  right->LQR.torque_setT[1];
  right->torque_send.Tw = -right->LQR.torque_setW;

  if (dbus->Remote.S2_u8 == 1)    // 离线
  {
    left->torque_send.T1 = 0.0f;
    left->torque_send.T2 = 0.0f;
    left->torque_send.Tw = 0.0f;
    right->torque_send.T1 = 0.0f;
    right->torque_send.T2 = 0.0f;
    right->torque_send.Tw = 0.0f;
  }
  else if (left->status.stand == 1 || right->status.stand == 1)   // 模拟倒地
  {
    left->torque_send.T1 = motor->left_front.PID_S.Output;
    left->torque_send.T2 = motor->left_back.PID_S.Output;
    left->torque_send.Tw = 0.0f;
    right->torque_send.T1 = motor->right_front.PID_S.Output;
    right->torque_send.T2 = motor->right_back.PID_S.Output;
    right->torque_send.Tw = 0.0f;
  }
}


void Chassis_GetStatus(Leg_Typedef *left, Leg_Typedef *right)
{ 
    // // 离地状态
    // if (fabs(left->LQR.Fn) <= 5.0f)
    // {
    //   left->status.offGround = 1;
    //   memcpy(left->LQR.K, ChassisL_LQR_K_fall, sizeof(float) * 12);
    // } else {
    //   left->status.offGround = 0;
    //   memcpy(left->LQR.K, ChassisL_LQR_K, sizeof(float) * 12);
    // }
    // if (fabs(right->LQR.Fn) <= 5.0f)
    // {
    //   right->status.offGround = 1;
    //   memcpy(right->LQR.K, ChassisR_LQR_K_fall, sizeof(float) * 12);
    // } else {
    //   right->status.offGround = 0;
    //   memcpy(right->LQR.K, ChassisR_LQR_K, sizeof(float) * 12);
    // }
    // memcpy(left->LQR.K, ChassisL_LQR_K, sizeof(float) * 12);
    // memcpy(right->LQR.K, ChassisR_LQR_K, sizeof(float) * 12);
    Chassis_Fit_K(ChassisL_LQR_K_coeffs, left->vmc_calc.L0[POS], left->LQR.K);
    Chassis_Fit_K(ChassisR_LQR_K_coeffs, right->vmc_calc.L0[POS], right->LQR.K);

    // 倒地自启
    // uint8_t is_fallen = (fabs(left->stateSpace.theta) >= 1.2f) || (fabs(right->stateSpace.theta) >= 1.2f);
    uint8_t is_fallen = (left->vmc_calc.L0[POS] >= 0.8f || right->vmc_calc.L0[POS] >= 0.8f) && (fabs(left->stateSpace.theta) >= 1.2f) || (fabs(right->stateSpace.theta) >= 1.2f);
    uint8_t can_recover = (fabs(left->stateSpace.theta) < 1.6f) && (fabs(right->stateSpace.theta) < 1.6f) && ((left->stateSpace.theta > 0) && (right->stateSpace.theta > 0));
    // 使用 left->status.stand 作为整车的状态标志 (0:正常, 1:倒地, 2:恢复)
    switch (left->status.stand)
    {
    case 0:   // 正常状态
      if (is_fallen)
      {
        left->status.stand = 1;
        right->status.stand = 1;
      }
      break;
    case 1:   // 倒地状态
      if (can_recover)
      {
        left->status.stand = 2;
        right->status.stand = 2;
      }
      // 否则保持倒地
      break;
    case 2:   // 恢复状态
      // if (is_fallen)  // 如果再次倒地，切回上一个状态
      // {
      //   left->status.stand = 1;
      //   right->status.stand = 1;
      // }
      // else
      // {
        left->status.stand_count++;
        right->status.stand_count++;
        if (left->status.stand_count >= 5000 || right->status.stand_count >= 5000)
        {
          left->status.stand_count = 0;
          right->status.stand_count = 0;
          left->status.stand = 0;
          right->status.stand = 0;
        }
      // }
      break;
    default:
      break;
    }
}

// 不同状态处理
void Chassis_StateHandle(Leg_Typedef *left, Leg_Typedef *right)
{
    int machine_state = left->status.stand;

    if (machine_state == 1) // 倒地
    {
      Chassis_Rotate(&ALL_MOTOR, left, right);
    }
    else if (machine_state == 2) // 恢复
    {
      // left->target.l0 = 0.06f;
      // right->target.l0 = 0.06f;
      // if (left->vmc_calc.L0[POS] <= 0.08f && right->vmc_calc.L0[POS] <= 0.08f)
      // {
      //   left->target.theta = 0.0f;
      //   right->target.theta = 0.0f;
      //   left->target.s = left->stateSpace.s;
      //   right->target.s = right->stateSpace.s;
      //   left->target.phi = 0;
      //   right->target.phi = 0;
      //   memcpy(&left->LQR.K, ChassisL_LQR_K_stand, sizeof(float) * 12);
      //   memcpy(&right->LQR.K, ChassisR_LQR_K_stand, sizeof(float) * 12);
      // }
      
      // 轮子给小，防止飞出
      // left->limit.W_max = 0.3f;
      // right->limit.W_max = 0.3f;
      // // 腿部正常
      // left->limit.T_max = 0;
      // right->limit.T_max = 0;
      Chassis_Rotate(&ALL_MOTOR, left, right);

    }
    else // 正常
    {
      left->limit.W_max = MAX_TORQUE_LEG_W;
      right->limit.W_max = MAX_TORQUE_LEG_W;
      left->limit.T_max = MAX_TORQUE_LEG_T;
      right->limit.T_max = MAX_TORQUE_LEG_T;
    }
}

// 获取目标值，使用规划
// 先一定速度旋转一定时间，后过零点后采用位置控制
static void getPIDAim(MOTOR_Typedef *motor, Leg_Typedef *left, Leg_Typedef *right)
{
  motor->left_front.DATA.aim = -1.0f; // 设定0.2rad/s
  if (left->stateSpace.theta >= 0.0f && left->stateSpace.theta <= 1.25f)
  {
    motor->left_front.DATA.aim = 0.0; // 最终位置
    motor->left_front.PID_S.Output = 0.0f;
  }
  motor->left_back.DATA.aim = -1.0f; // 设定0.2rad/s
  if (left->stateSpace.theta >= 0.0f && left->stateSpace.theta <= 1.25f)
  {
    motor->left_back.DATA.aim = 0.0f; // 最终位置
    motor->left_back.PID_S.Output = 0.0f;
  }
  motor->right_front.DATA.aim = 1.0f; // 设定0.2rad/s
  if (right->stateSpace.theta >= 0.0f && right->stateSpace.theta <= 1.25f)
  {
    motor->right_front.DATA.aim = 0.0f; // 最终位置
    motor->right_front.PID_S.Output = 0.0f;
  }
  motor->right_back.DATA.aim = 1.0f; // 设定0.2rad/s
  if (right->stateSpace.theta >= 0.0f && right->stateSpace.theta <= 1.25f)
  {
    motor->right_back.DATA.aim = 0.0f; // 最终位置
    motor->right_back.PID_S.Output = 0.0f;
  }
}

// 正常后清除目标值
static void ClearAim(MOTOR_Typedef *motor)
{
  motor->left_front.DATA.aim = 0.0f;
  motor->right_front.DATA.aim = 0.0f;
  motor->left_back.DATA.aim = 0.0f;
  motor->right_back.DATA.aim = 0.0f;
}


// 倒地后旋转腿，采用pid测试
void Chassis_Rotate(MOTOR_Typedef *motor, Leg_Typedef *left, Leg_Typedef *right)
{
  getPIDAim(motor, left, right);
  PID_Calculate(&motor->left_front.PID_S, motor->left_front.DATA.vel, motor->left_front.DATA.aim);
  PID_Calculate(&motor->left_back.PID_S, motor->left_back.DATA.vel, motor->left_back.DATA.aim);
  PID_Calculate(&motor->right_front.PID_S, motor->right_front.DATA.vel, motor->right_front.DATA.aim);
  PID_Calculate(&motor->right_back.PID_S, motor->right_back.DATA.vel, motor->right_back.DATA.aim);

  // mit_ctrl(&hcan1, LEG_LF+1, 0, 0, 0, 0, motor->left_front.PID_S.Output);
  // mit_ctrl(&hcan1, LEG_LB+1, 0, 0, 0, 0, motor->left_back.PID_S.Output);
  // mit_ctrl(&hcan1, LEG_RF+1, 0, 0, 0, 0, motor->right_front.PID_S.Output);
  // mit_ctrl(&hcan1, LEG_RB+1, 0, 0, 0, 0, motor->right_back.PID_S.Output);

}

// 速度规划，用于获取速度实际值
float Chassis_SpeedPlan(float speed_target, float a_max, float T, float dt, Leg_Typedef *left, Leg_Typedef *right)
{
  static float t = 0.0f;  // 用于总时间
  if (fabsf(speed_target - left->stateSpace.dot_s) < 0.01f && fabsf(speed_target - right->stateSpace.dot_s) < 0.01f)
  {
    t = 0.0f;
    return speed_target;
  }

  float speed_out = 0.0f, k = 0.0f;
  if (speed_target > 0)
  {
    t += dt;
    k = a_max / T;
    speed_out = -1.0f / 2.0f * k * k * t * t + a_max * t;
    return speed_out;
  }
  if (speed_target < 0)
  {
    t += dt;
    k = a_max / T;
    speed_out = 1.0f / 2.0f * k * k * t * t - a_max * t;
    return speed_out;
  }
}