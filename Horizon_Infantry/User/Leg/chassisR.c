#include "chassisR.h"
#include "MY_Define.h"
#include "DM_Motor.h"

void ChassisR_Init(void)
{
    motor_mode(&hcan1, LEG_RF+1, 0x000, 0xfc);
    osDelay(1);
    motor_mode(&hcan1, LEG_RB+1, 0x000, 0xfc);
    osDelay(1);
}
