#include "chassisR.h"
#include "MY_Define.h"
#include "DM_Motor.h"
#include "All_Init.h"
#include "vmc.h"

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
    object->stateSpace.theta = -PI / 2.0f + object->vmc_calc.phi0[POS] + imu->pitch /  57.3f;
    object->stateSpace.dtheta = Discreteness_Diff(&object->Discreteness.Theta, object->stateSpace.theta, dt);
    object->stateSpace.phi = imu->pitch / 57.3f;
    object->stateSpace.dphi = Discreteness_Diff(&object->Discreteness.Phi, object->stateSpace.phi, dt);

    object->stateSpace.ddtheta = Discreteness_Diff(&object->Discreteness.dTheta, object->stateSpace.dtheta, dt);
}