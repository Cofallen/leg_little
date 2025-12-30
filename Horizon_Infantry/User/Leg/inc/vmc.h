#ifndef __VMC_H
#define __VMC_H

#include "main.h"
#include "mymath.h"
#include "pid_temp.h"
#include "arm_math.h"
#include "Motors.h"
#include "IMU_Task.h"

typedef struct 
{
    float phi0[3];
    float L0[3];
    float phi1[3];
    float phi2[3];
    float phi3[3];
    float phi4[3];
}vmc_Typedef;                   // vmc正解计算中间变量

typedef struct 
{
    struct 
    {
        float theta;
        float dtheta;
        float s;
        float dot_s;
        float phi;
        float dphi;
    }stateSpace;                          // 状态变量
    
    struct
    {
        float l0;                     // 目标腿长
        float roll;                   // roll 补偿
        float yaw;                    // 转向控制
        float d2theta;                // 目标劈叉角度
    }target;

    vmc_Typedef vmc_calc;

    struct 
    {
        Discreteness_TypeDef Phi_1;   // vmc正解phi1 求解速度
        Discreteness_TypeDef Phi_4;   // vmc正解phi4 求解速度
        Discreteness_TypeDef L;       // vmc正解L    求解拟合杆速度
        Discreteness_TypeDef D_L;     // vmc正解D_L  求解拟合杆加速度
        Discreteness_TypeDef Theta;   // vmc正解Theta 求解速度
        Discreteness_TypeDef dTheta;  // vmc正解dTheta 求解加速度
        Discreteness_TypeDef Theta_w;     // 轮子差分 
    }vmc_Discreteness;
    
    struct 
    {
        pid_type_def F0_l;
        pid_type_def Yaw;
        pid_type_def Delta;
    }pid;

    struct 
    {
        uint8_t offGround;
        uint8_t jump;
        uint8_t stand;
        uint8_t crouch;
    }status;
    
}Leg_Typedef;


extern Leg_Typedef Leg_l;
extern Leg_Typedef Leg_r;

void Vmc_Init(Leg_Typedef *object, float target_l0);
void Vmc_calc(Leg_Typedef *object, MOTOR_Typedef *motor, IMU_Data_t *imu, float dt);
static void getPhi(vmc_Typedef *vmc, float phi1, float phi4, float l1, float l2, float l3, float l4, float l5);


#endif // !__VMC_H