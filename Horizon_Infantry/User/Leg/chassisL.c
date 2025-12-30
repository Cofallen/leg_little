#include "chassisL.h"
#include "MY_Define.h"
#include "DM_Motor.h"
#include "controller.h"
#include "All_Init.h"

void ChassisL_Init(void)
{
    motor_mode(&hcan1, LEG_LF+1, 0x000, 0xfc);
    osDelay(1);
    motor_mode(&hcan1, LEG_LB+1, 0x000, 0xfc);
    osDelay(1);
}
