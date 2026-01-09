#include "vmc.h"
#include "pid_temp.h"
#include "Motors.h"
#include "IMU_task.h"

// 应放在对应文件内
Leg_Typedef Leg_l;
Leg_Typedef Leg_r;

// #define MATH_PI 3.14159265358979323846f


void Vmc_Init(Leg_Typedef *object, float target_l0)
{
    object->Discreteness.L.last_diff = BANDENG_LENGTH; 
    object->Discreteness.Phi_1.last_diff = PI;        // 零点
    object->Discreteness.Phi_4.last_diff = 0;

    object->target.l0 = target_l0;
    object->target.roll = 0.0f;
    object->target.yaw = 0.0f;
    object->target.d2theta = 0.0f;

    // const float F0_control[3] = {8000.0f, 0.0f, 30000.0f};  // 下落状态切换
    const float F0_control[3] = {800.0f, 1.0f, 5000.0f};
    const float Yaw_control[3] = {1.0f, 0.0f, 300.0f};
    const float Delta_control[3] = {10.0f, 0.0f, 10.0f};
    const float Roll_control[3] = {1.0f, 0.0f, 0.0f};

    PID_init(&object->pid.F0_l, PID_POSITION, F0_control, 30.0f, 0.0f);
    PID_init(&object->pid.Yaw, PID_POSITION, Yaw_control, 0.5f, 0.0f);
    PID_init(&object->pid.Delta, PID_POSITION, Delta_control, 5.0f, 0.0f);
    PID_init(&object->pid.Roll, PID_POSITION, Roll_control, 0.4f, 0.0f);
}

void Vmc_calcL(Leg_Typedef *object, MOTOR_Typedef *motor, IMU_Data_t *imu, float dt)
{
    // 基本状态获取
    // object->stateSpace.dphi = imu->gyro[0];  // 这个是什么，我算的是我的差分运算时对的，这个整体趋势对，不像角加速度
    // 上面写错了，是状态变量更新函数

    // 左腿
    object->vmc_calc.phi1[POS] = PI - motor->left_front.DATA.pos;
    object->vmc_calc.phi4[POS] = 0  - motor->left_back.DATA.pos;
    
    getPhi(&object->vmc_calc, object->vmc_calc.phi1[POS], object->vmc_calc.phi4[POS], L1_LENGTH, L2_LENGTH, L3_LENGTH, L4_LENGTH, L5_LENGTH);

    object->vmc_calc.L0[VEL] = Discreteness_Diff(&object->Discreteness.L, object->vmc_calc.L0[POS], dt);
    object->vmc_calc.L0[ACC] = Discreteness_Diff(&object->Discreteness.D_L, object->vmc_calc.L0[VEL], dt);
    
    getMatJRM(&object->vmc_calc, object->vmc_calc.phi0[POS], object->vmc_calc.phi1[POS], object->vmc_calc.phi2[POS], object->vmc_calc.phi3[POS], object->vmc_calc.phi4[POS], object->vmc_calc.L0[POS], L1_LENGTH, L4_LENGTH);
    Vmc_getFnL(object, imu);
}


void Vmc_calcR(Leg_Typedef *object, MOTOR_Typedef *motor, IMU_Data_t *imu, float dt)
{
    // 基本状态获取
    // object->stateSpace.dphi = imu->gyro[0];  // 这个是什么，我算的是我的差分运算时对的，这个整体趋势对，不像角加速度
    // 上面写错了，是状态变量更新函数

    // 右腿
    object->vmc_calc.phi1[POS] = PI - motor->right_back.DATA.pos;      // 注意ID
    object->vmc_calc.phi4[POS] = 0  - motor->right_front.DATA.pos;

    getPhi(&object->vmc_calc, object->vmc_calc.phi1[POS], object->vmc_calc.phi4[POS], L1_LENGTH, L2_LENGTH, L3_LENGTH, L4_LENGTH, L5_LENGTH);

    object->vmc_calc.L0[VEL] = Discreteness_Diff(&object->Discreteness.L, object->vmc_calc.L0[POS], dt);
    object->vmc_calc.L0[ACC] = Discreteness_Diff(&object->Discreteness.D_L, object->vmc_calc.L0[VEL], dt);

    getMatJRM(&object->vmc_calc, object->vmc_calc.phi0[POS], object->vmc_calc.phi1[POS], object->vmc_calc.phi2[POS], object->vmc_calc.phi3[POS], object->vmc_calc.phi4[POS], object->vmc_calc.L0[POS], L1_LENGTH, L4_LENGTH);
    Vmc_getFnR(object, imu);
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

static void getMatJRM(vmc_Typedef *vmc, float phi0, float phi1, float phi2, float phi3, float phi4, float L0, float l1, float l4)
{
    vmc->JRM[0][0] = -l1 * arm_sin_f32(phi0 - phi3) * arm_sin_f32(phi1 - phi2) / arm_sin_f32(phi2 - phi3);
    vmc->JRM[0][1] = -l1 * arm_sin_f32(phi1 - phi2) * arm_cos_f32(phi0 - phi3) / (L0 * arm_sin_f32(phi2 - phi3));
    vmc->JRM[1][0] = -l4 * arm_sin_f32(phi0 - phi2) * arm_sin_f32(phi3 - phi4) / arm_sin_f32(phi2 - phi3);
    vmc->JRM[1][1] = -l4 * arm_sin_f32(phi3 - phi4) * arm_cos_f32(phi0 - phi2) / (L0 * arm_sin_f32(phi2 - phi3));
}

static float Vmc_getFnL(Leg_Typedef *object, IMU_Data_t *imu)
{
    float P = 0.0f, ddz_w = 0.0f;
    static float ddl_fL, ddtheta_fL, acc_fL; 
    ddl_fL = Lowpass_Filter(&ddl_fL, object->vmc_calc.L0[ACC], 0.1f);
    acc_fL = Lowpass_Filter(&acc_fL, imu->accel[2], 0.1f);

    P = object->LQR.F_0 * arm_cos_f32(object->stateSpace.theta) + object->LQR.T_p * arm_sin_f32(object->stateSpace.theta) / object->vmc_calc.L0[POS];
    ddz_w = (acc_fL - 9.81f) - ddl_fL * arm_cos_f32(object->stateSpace.theta) + 2.0f * object->vmc_calc.L0[VEL] * object->stateSpace.dtheta * arm_sin_f32(object->stateSpace.theta) + \
            object->vmc_calc.L0[POS] * arm_sin_f32(object->stateSpace.theta) * ddtheta_fL + object->vmc_calc.L0[POS] * object->stateSpace.dtheta * object->stateSpace.dtheta * arm_cos_f32(object->stateSpace.theta);
    object->LQR.Fn = P + MASS_WHEEL * 9.81f + MASS_WHEEL * ddz_w;
}

static float Vmc_getFnR(Leg_Typedef *object, IMU_Data_t *imu)
{
    float P = 0.0f, ddz_w = 0.0f;
    static float ddl_fR, ddtheta_fR, acc_fR; 
    ddl_fR = Lowpass_Filter(&ddl_fR, object->vmc_calc.L0[ACC], 0.1f);
    acc_fR = Lowpass_Filter(&acc_fR, imu->accel[2], 0.1f);

    P = object->LQR.F_0 * arm_cos_f32(object->stateSpace.theta) + object->LQR.T_p * arm_sin_f32(object->stateSpace.theta) / object->vmc_calc.L0[POS];
    ddz_w = (acc_fR - 9.81f) - ddl_fR * arm_cos_f32(object->stateSpace.theta) + 2.0f * object->vmc_calc.L0[VEL] * object->stateSpace.dtheta * arm_sin_f32(object->stateSpace.theta) + \
            object->vmc_calc.L0[POS] * arm_sin_f32(object->stateSpace.theta) * ddtheta_fR + object->vmc_calc.L0[POS] * object->stateSpace.dtheta * object->stateSpace.dtheta * arm_cos_f32(object->stateSpace.theta);
    object->LQR.Fn = P + MASS_WHEEL * 9.81f + MASS_WHEEL * ddz_w;
}