#include "vmc.h"
#include "pid_temp.h"
#include "Motors.h"
#include "IMU_task.h"

// 应放在对应文件内
Leg_Typedef Leg_l;
Leg_Typedef Leg_r;

// #define MATH_PI 3.14159265358979323846f
#define BANDENG_LENGTH 8.5f

#define RADIUS_WHEEL   0.058f
#define MASS_WHEEL     0.5f
#define MASS_BODY     1.0f
#define L1_LENGTH     0.105f
#define L2_LENGTH     0.13f
#define L3_LENGTH     0.13f
#define L4_LENGTH     0.105f
#define L5_LENGTH     0.0f

#define POS 0
#define VEL 1
#define ACC 2

void Vmc_Init(Leg_Typedef *object, float target_l0)
{
    object->vmc_Discreteness.L.last_diff = BANDENG_LENGTH; 
    object->vmc_Discreteness.Phi_1.last_diff = PI;        // 零点
    object->vmc_Discreteness.Phi_4.last_diff = 0;

    object->target.l0 = target_l0;
    object->target.roll = 0.0f;
    object->target.yaw = 0.0f;
    object->target.d2theta = 0.0f;

    const float F0_control[3] = {100.0f, 0.0f, 200.0f};
    const float Yaw_control[3] = {0.0f, 0.0f, 0.0f};
    const float Delta_control[3] = {0.0f, 0.0f, 0.0f};

    PID_init(&object->pid.F0_l, 0, F0_control, 100.0f, 0.0f);
    PID_init(&object->pid.Yaw, 0, Yaw_control, 100.0f, 0.0f);
    PID_init(&object->pid.Delta, 0, Delta_control, 100.0f, 0.0f);
}

void Vmc_calc(Leg_Typedef *object, MOTOR_Typedef *motor, IMU_Data_t *imu, float dt)
{
    // 基本状态获取
    // object->stateSpace.dphi = imu->gyro[0];  // 这个是什么，我算的是我的差分运算时对的，这个整体趋势对，不像角加速度
    object->stateSpace.phi = imu->pitch;        // 注意转弧度
    object->stateSpace.dphi = Discreteness_Diff(&object->vmc_Discreteness.Phi_1, object->stateSpace.phi, dt);
    // 上面写错了，是状态变量更新函数

    // 左腿
    object->vmc_calc.phi1[POS] = PI - motor->left_front.DATA.pos;
    object->vmc_calc.phi4[POS] = 0  - motor->left_back.DATA.pos;

    getPhi(&object->vmc_calc, object->vmc_calc.phi1[POS], object->vmc_calc.phi4[POS], L1_LENGTH, L2_LENGTH, L3_LENGTH, L4_LENGTH, L5_LENGTH);
}




static void getPhi(vmc_Typedef *vmc, float phi1, float phi4, float l1, float l2, float l3, float l4, float l5)
{
    float x_B = 0.0f, y_B = 0.0f, x_C = 0.0f, y_C = 0.0f, x_D = 0.0f, y_D = 0.0f;
    float A0 = 0.0f, B0 = 0.0f, C0 = 0.0f;
    float l_BD = 0.0f;
    
    x_B = -l5 / 2.0f + arm_cos_f32(phi1) * l1;
    y_B = arm_sin_f32(phi1) * l1;

    x_D =  l5 / 2.0f + arm_cos_f32(phi4) * l4;
    y_D = arm_sin_f32(phi4) * l4;

    A0 = 2 * l2 * (x_D - x_B);
    B0 = 2 * l2 * (y_D - y_B);
    l_BD = sqrtf(((x_D - x_B) * (x_D - x_B) + (y_D - y_B) * (y_D - y_B)));
    C0 = l2 * l2 + l_BD * l_BD - l3 * l3;

    vmc->phi2[POS] = 2.0f * atan2(B0 + sqrt(A0 * A0 + B0 * B0 - C0 * C0), A0 + C0);

    x_C = -l5 / 2.0f + l1 * arm_cos_f32(phi1) + l2 * arm_cos_f32(vmc->phi2[POS]);
    y_C =      l1 * arm_sin_f32(phi1) + l2 * arm_sin_f32(vmc->phi2[POS]);

    vmc->phi3[POS] = atan2f(y_C - y_D, x_C - x_D);

    vmc->L0[POS] = sqrtf((x_C * x_C) + (y_C * y_C));
    vmc->phi0[POS] = atan2f(y_C, x_C);
}