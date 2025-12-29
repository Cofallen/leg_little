#include "Gimbal_Task.h"

/************************************************************万能分隔符**************************************************************
 * 	@author:			//小瑞
 *	@performance:	    //头部PID+前馈总初始化函数
 *	@parameter:		    //
 *	@time:				//23-04-13 12:42
 *	@ReadMe:			//
 ************************************************************万能分隔符**************************************************************/
uint8_t MOTOR_PID_Gimbal_INIT(MOTOR_Typedef *MOTOR)
{
    //云台电机初始化
    float PID_F_Pitch[3] = {   1.0f,   0.0f,   0.0f   };
    float PID_P_Pitch[3] = {   1.0f,   0.0f,   0.0f   };
    float PID_S_Pitch[3] = {   1.0f,   0.0f,   0.0f   };
    Feedforward_Init(&MOTOR->DJI_6020_Pitch.PID_F, 3000, PID_F_Pitch,
                     0.5f, 2, 2);
    PID_Init(&MOTOR->DJI_6020_Pitch.PID_P, 6000.0f, 2000.0f,
             PID_P_Pitch, 1000.0f, 1000.0f,
             0.7f, 0.7f, 2,
             Integral_Limit|OutputFilter|ErrorHandle//积分限幅,输出滤波,堵转监测
             |Trapezoid_Intergral|ChangingIntegrationRate//梯形积分,变速积分
             |Derivative_On_Measurement|DerivativeFilter);//微分先行,微分滤波器
    PID_Init(&MOTOR->DJI_6020_Pitch.PID_S, 6000.0f, 2000.0f,
             PID_S_Pitch, 1000.0f, 1000.0f,
             0.7f, 0.7f, 2,
             Integral_Limit|OutputFilter|ErrorHandle//积分限幅,输出滤波,堵转监测
             |Trapezoid_Intergral|ChangingIntegrationRate//梯形积分,变速积分
             |Derivative_On_Measurement|DerivativeFilter);//微分先行,微分滤波器

    float PID_F_Yaw[3] = {   1.0f,   0.0f,   0.0f   };
    float PID_P_Yaw[3] = {   1.0f,   0.0f,   0.0f   };
    float PID_S_Yaw[3] = {   1.0f,   0.0f,   0.0f   };
    Feedforward_Init(&MOTOR->DJI_6020_Yaw.PID_F, 3000, PID_F_Yaw,
                     0.5f, 2, 2);
    PID_Init(&MOTOR->DJI_6020_Yaw.PID_P, 6000.0f, 2000.0f,
             PID_P_Yaw, 1000.0f, 1000.0f,
             0.7f, 0.7f, 2,
             Integral_Limit|OutputFilter|ErrorHandle//积分限幅,输出滤波,堵转监测
             |Trapezoid_Intergral|ChangingIntegrationRate//梯形积分,变速积分
             |Derivative_On_Measurement|DerivativeFilter);//微分先行,微分滤波器
    PID_Init(&MOTOR->DJI_6020_Yaw.PID_S, 6000.0f, 2000.0f,
             PID_S_Yaw, 1000.0f, 1000.0f,
             0.7f, 0.7f, 2,
             Integral_Limit|OutputFilter|ErrorHandle//积分限幅,输出滤波,堵转监测
             |Trapezoid_Intergral|ChangingIntegrationRate//梯形积分,变速积分
             |Derivative_On_Measurement|DerivativeFilter);//微分先行,微分滤波器

    return RUI_DF_READY;
}

uint8_t Gimbal_AIM_INIT(RUI_ROOT_STATUS_Typedef *Root, MOTOR_Typedef *MOTOR)
{
    //检查离线
    if (Root->MOTOR_HEAD_Pitch     == RUI_DF_OFFLINE ||
        Root->MOTOR_HEAD_Yaw      == RUI_DF_OFFLINE)
        return RUI_DF_ERROR;

    //电机清空
    RUI_F_HEAD_MOTOR_CLEAR(&MOTOR->DJI_6020_Pitch, 1);
    RUI_F_HEAD_MOTOR_CLEAR(&MOTOR->DJI_6020_Yaw, 1);

    return RUI_DF_READY;
}

uint8_t gimbal_task(CONTAL_Typedef *CONTAL,
                    RUI_ROOT_STATUS_Typedef *Root,
                    MOTOR_Typedef *MOTOR,
                    IMU_Data_t *IMU)
{
    static uint8_t PID_INIT = RUI_DF_ERROR;
    static uint8_t AIM_INIT = RUI_DF_ERROR;

    //电机PID赋值
    if (PID_INIT != RUI_DF_READY)
    {
      PID_INIT = MOTOR_PID_Gimbal_INIT(MOTOR);
      return RUI_DF_ERROR;
    }

    /*电机在线检测*/
    if (AIM_INIT != RUI_DF_READY)
    {
      AIM_INIT = Gimbal_AIM_INIT(Root, MOTOR);
      return RUI_DF_ERROR;
    }

    /*底盘跟随变量赋值*/
    CONTAL->CG.RELATIVE_ANGLE = (int16_t) (CONTAL->CG.YAW_INIT_ANGLE - 4096 - MOTOR->DJI_6020_Yaw.DATA.Angle_now);
    CONTAL->CG.YAW_SPEED =  MOTOR->DJI_6020_Yaw.DATA.Speed_now;
    if (CONTAL->CG.RELATIVE_ANGLE > 4096)
    {
        CONTAL->CG.RELATIVE_ANGLE -= 8192;
    }
    else if (CONTAL->CG.RELATIVE_ANGLE < -4096)
    {
        CONTAL->CG.RELATIVE_ANGLE += 8192;
    }

    /*目标值赋值*/
    MOTOR->DJI_6020_Pitch.DATA.Aim = CONTAL->HEAD.Pitch;
    MOTOR->DJI_6020_Yaw.DATA.Aim = CONTAL->HEAD.Yaw;

    if(CONTAL->MOD[0] - CONTAL->MOD[1] == 1)//自瞄模式
    {
        float PID_P_Pitch[3] = {   1.0f,   0.0f,   0.0f   };
        float PID_S_Pitch[3] = {   1.0f,   0.0f,   0.0f   };
        PID_set(&MOTOR->DJI_6020_Pitch.PID_P, PID_P_Pitch);
        PID_set(&MOTOR->DJI_6020_Pitch.PID_S, PID_S_Pitch);
    }
    if(CONTAL->MOD[0] - CONTAL->MOD[1] <= 0)//手瞄模式
    {
        float PID_P_Pitch[3] = {   1.0f,   0.0f,   0.0f   };
        float PID_S_Pitch[3] = {   1.0f,   0.0f,   0.0f   };
        PID_set(&MOTOR->DJI_6020_Pitch.PID_P, PID_P_Pitch);
        PID_set(&MOTOR->DJI_6020_Pitch.PID_S, PID_S_Pitch);
    }

    CONTAL->MOD[1] = CONTAL->MOD[0];

    /*遥控离线保护*/
    if(!Root->RM_DBUS)
    {
        MOTOR->DJI_6020_Pitch.PID_P.IntegralLimit = 0;
        MOTOR->DJI_6020_Pitch.PID_S.IntegralLimit = 0;
        MOTOR->DJI_6020_Pitch.DATA.Aim = (float)MOTOR->DJI_6020_Pitch.DATA.Angle_Infinite;

        MOTOR->DJI_6020_Yaw.PID_P.IntegralLimit = 0;
        MOTOR->DJI_6020_Yaw.PID_S.IntegralLimit = 0;
        MOTOR->DJI_6020_Yaw.DATA.Aim = (float)MOTOR->DJI_6020_Yaw.DATA.Angle_Infinite;

        PID_INIT = RUI_DF_ERROR;
        AIM_INIT = RUI_DF_ERROR;
    }

    /*堵转处理*/
    RUI_F_HEAD_MOTOR3508_STUCK(&MOTOR->DJI_6020_Pitch, 4000, 10);
    RUI_F_HEAD_MOTOR3508_STUCK(&MOTOR->DJI_6020_Yaw, 4000, 10);

    /*Pitch计算*/
    Feedforward_Calculate(&MOTOR->DJI_6020_Pitch.PID_F,
                          MOTOR->DJI_6020_Pitch.DATA.Aim);
    PID_Calculate(&MOTOR->DJI_6020_Pitch.PID_P,
                  IMU->pitch * 22.75555555555556f,
                  MOTOR->DJI_6020_Pitch.DATA.Aim);
    PID_Calculate(&MOTOR->DJI_6020_Pitch.PID_S,
                  IMU->gyro[1] * 100.0f,
                  MOTOR->DJI_6020_Pitch.PID_P.Output);

    /*Yaw计算*/
    Feedforward_Calculate(&MOTOR->DJI_6020_Yaw.PID_F,
                          MOTOR->DJI_6020_Yaw.DATA.Aim);
    PID_Calculate(&MOTOR->DJI_6020_Yaw.PID_P,
                  IMU->YawTotalAngle * 22.75555555555556f,
                  MOTOR->DJI_6020_Yaw.DATA.Aim);
    PID_Calculate(&MOTOR->DJI_6020_Yaw.PID_S,
                  IMU->gyro[2] * 100.0f,
                  MOTOR->DJI_6020_Yaw.PID_P.Output);

    /*总输出计算*/
    float tmp_G[2];

    tmp_G[0] = MOTOR->DJI_6020_Pitch.PID_F.Output +
               MOTOR->DJI_6020_Pitch.PID_S.Output;

    tmp_G[1] = MOTOR->DJI_6020_Yaw.PID_F.Output +
               MOTOR->DJI_6020_Yaw.PID_S.Output;

    /*CAN发送*/
    DJI_Current_Ctrl(&hcan1,
                     0x1FF,
                     (int16_t)tmp_G[0],
                     (int16_t)tmp_G[1],
                     0,
                     0);

    return RUI_DF_READY;
}
