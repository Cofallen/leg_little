/*
 * @Author: 王举人 11546637+wang-juren@user.noreply.gitee.com
 * @Date: 2024-11-12 20:51:02
 * @LastEditors: 王举人 11546637+wang-juren@user.noreply.gitee.com
 * @LastEditTime: 2024-11-16 10:21:36
 * @FilePath: \Horizon_Infantry\User\App\WHW_IRQN.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/*
 * @Author: 王举人 11546637+wang-juren@user.noreply.gitee.com
 * @Date: 2024-11-12 20:51:02
 * @LastEditors: 王举人 11546637+wang-juren@user.noreply.gitee.com
 * @LastEditTime: 2024-11-15 22:47:11
 * @FilePath: \Horizon_Infantry\User\App\WHW_IRQN.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/*
 *                        _oo0oo_
 *                       o8888888o
 *                       88" . "88
 *                       (| -_- |)
 *                       0\  =  /0
 *                     ___/`---'\___
 *                   .' \\|     |// '.
 *                  / \\|||  :  |||// \
 *                 / _||||| -:- |||||- \
 *                |   | \\\  - /// |   |
 *                | \_|  ''\---/''  |_/ |
 *                \  .-\__  '-'  ___/-. /
 *              ___'. .'  /--.--\  `. .'___
 *           ."" '<  `.___\_<|>_/___.' >' "".
 *          | | :  `- \`.;`\ _ /`;.`/ - ` : | |
 *          \  \ `_.   \_ __\ /__ _/   .-` /  /
 *      =====`-.____`.___ \_____/___.-`___.-'=====
 *                        `=---='
 * 
 * 
 *      ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * 
 *            佛祖保佑     永不宕机     永无BUG
 * 
 *        佛曰:  
 *                写字楼里写字间，写字间里程序员；  
 *                程序人员写程序，又拿程序换酒钱。  
 *                酒醒只在网上坐，酒醉还来网下眠；  
 *                酒醉酒醒日复日，网上网下年复年。  
 *                但愿老死电脑间，不愿鞠躬老板前；  
 *                奔驰宝马贵者趣，公交自行程序员。  
 *                别人笑我忒疯癫，我笑自己命太贱；  
 *                不见满街漂亮妹，哪个归得程序员？
 */

/*
 * @Descripttion: 
 * @version: 
 * @Author: Eugene
 * @Date: 2024-07-04 15:42:35
 * @LastEditors: Andy
 * @LastEditTime: 2024-07-07 11:27:00
 */

#include "WHW_IRQN.h"
#include "VOFA.h"
#include "chassisL.h"
#include "chassisR.h"
#include "vmc.h"

uint8_t move_G, move_S, move_C, move_P;
float t1,t2,dt;
static uint8_t TX[12] = {0xff,0xf1,0xfd,0x90,0x86,0xa7,0xff,0xf1,0xfd,0x90,0x86,0xa7};

//34ms,画UI任务
void StartRobotUITask(void const * argument)
{
    portTickType currentTimeRobotUI;
    currentTimeRobotUI = xTaskGetTickCount();

    for (;;)
    {
        osDelayUntil(&currentTimeRobotUI, 40);
    }
}

//运动控制任务
void StartMoveTask(void const * argument)
{
    portTickType currentTimeMove;
    currentTimeMove = xTaskGetTickCount();

    osDelay(10000);
    ChassisL_Init();
    ChassisR_Init();
    Vmc_Init(&Leg_l, 0.11f);
    Vmc_Init(&Leg_r, 0.11f);

    while (IMU_Data.pitch == 0.0f)
    {
        osDelay(1);
    }
    osDelay(3000); // 等待IMU数据稳定
    for (;;)
    {
        RUI_V_CONTAL.DWT_TIME.Move_Dtime = DWT_GetDeltaT(&RUI_V_CONTAL.DWT_TIME.Move_DWT_Count);
        Vmc_calcL(&Leg_l, &ALL_MOTOR, &IMU_Data, RUI_V_CONTAL.DWT_TIME.Move_Dtime);
        Vmc_calcR(&Leg_r, &ALL_MOTOR, &IMU_Data, RUI_V_CONTAL.DWT_TIME.Move_Dtime);
        ChassisL_UpdateState(&Leg_l, &ALL_MOTOR, &IMU_Data, RUI_V_CONTAL.DWT_TIME.Move_Dtime);
        ChassisR_UpdateState(&Leg_r, &ALL_MOTOR, &IMU_Data, RUI_V_CONTAL.DWT_TIME.Move_Dtime);
        Chassis_UpdateStateS(&Leg_l, &Leg_r, &ALL_MOTOR, RUI_V_CONTAL.DWT_TIME.Move_Dtime);
        ChassisL_Control(&Leg_l, &WHW_V_DBUS, &IMU_Data, RUI_V_CONTAL.DWT_TIME.Move_Dtime);
        ChassisR_Control(&Leg_r, &WHW_V_DBUS, &IMU_Data, RUI_V_CONTAL.DWT_TIME.Move_Dtime);
        // VOFA_justfloat(RUI_V_CONTAL.DWT_TIME.Move_Dtime,
        //                RUI_V_CONTAL.DWT_TIME.IMU_Dtime,
        //                RUI_V_CONTAL.DWT_TIME.TIM7_Dtime,
        //                0,Leg_l.stateSpace.phi,Leg_l.stateSpace.dphi,Leg_l.Discreteness.Phi_1.diff_num,0,
        //                Leg_l.vmc_calc.L0[0],Leg_l.vmc_calc.phi0[0]);
        VOFA_justfloat(RUI_V_CONTAL.DWT_TIME.Move_Dtime,
                       RUI_V_CONTAL.DWT_TIME.TIM7_Dtime,
                       Leg_l.stateSpace.theta,
                        Leg_l.stateSpace.s,
                        Leg_l.stateSpace.phi,
                        Leg_r.stateSpace.theta,
                        Leg_r.stateSpace.s,
                        Leg_r.stateSpace.phi, Leg_l.LQR.torque_setW, Leg_r.LQR.torque_setW);

        osDelay(1);
    }
}

//对抗控制任务(电容,发射)
void StartDefiantTask(void const * argument)
{
    portTickType currentTimeDefiant;
    currentTimeDefiant = xTaskGetTickCount();

    for(;;)
    {
        osDelayUntil(&currentTimeDefiant, 1);
    }
}

//陀螺仪解算与自瞄发送任务
void StartIMUTask(void const * argument)
{
    portTickType currentTimeIMU;
    currentTimeIMU = xTaskGetTickCount();

    static uint32_t dt_pc = 0;
    static uint32_t INS_DWT_Count = 0;

    //陀螺仪初始化
    const float imu_temp_PID[3] = TEMPERATURE_PID;
    PID_init(&imu_temp, PID_POSITION, imu_temp_PID,
             TEMPERATURE_PID_MAX_OUT, TEMPERATURE_PID_MAX_IOUT);
    IMU_QuaternionEKF_Init(10, 0.001f, 10000000, 1, 0.001f,0); //ekf初始化
    HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
    while(BMI088_init()){}

    for(;;)
    {
        INS_Task(&IMU_Data, &imu_temp);
        // dt_pc = (uint32_t)DWT_GetDeltaT(&INS_DWT_Count);
        RUI_V_CONTAL.DWT_TIME.IMU_Dtime = DWT_GetDeltaT(&RUI_V_CONTAL.DWT_TIME.IMU_DWT_Count);
        // CONTROL_VISION_SendData(IMU_Data.pitch, IMU_Data.yaw,
        //                          RUI_V_CONTAL.DWT_TIME.IMU_Dtime, 1, 1);
        // Vision_Tx_Data(IMU_Data.pitch, IMU_Data.yaw,
        //                dt_pc, 1, 1);

        osDelayUntil(&currentTimeIMU, 1);
    }
}

//整车监控任务
void StartRootTask(void const * argument)
{
    portTickType currentTimeRoot;
    currentTimeRoot = xTaskGetTickCount();

    // //使用基准电压来校准
    // init_vrefint_reciprocal();

    for(;;)
    {
        // RUI_F_ROOT(&RUI_ROOT_STATUS, &WHW_V_DBUS, &ALL_MOTOR, &CAPDATE.GET);
        // voltage = get_battery_voltage();

        osDelayUntil(&currentTimeRoot, 5);
    }
}

void BSP_TIM_IRQHandler(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM7) {
		TX[0]++;
		CANSPI_SEND(&hspi2, 0x201, TX);
	}
}

/************************************************************万能分隔符**************************************************************
 * 	@author:			//小瑞
 *	@performance:	    //CAN接收函数
 *	@parameter:		    //
 *	@time:				//22-11-23 20:42
 *	@ReadMe:			//
 *  @LastUpDataTime:    //2023-04-20 02:52    bestrui
 *  @UpData：           //更新成共用体
 *  @LastUpDataTime:    //2023-05-06 20:23    bestrui
 *  @UpData：           //更新判断逻辑
 ************************************************************万能分隔符**************************************************************/
uint8_t test[8];
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
	CAN_RxHeaderTypeDef can_rx;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan , CAN_RX_FIFO0 , &can_rx , rx_data);
    
	if (hcan == &hcan1)		
	{
		//CAN1
		switch (can_rx.StdId)
		{
            case 0x11 + LEG_LF://拨弹
                dm4310_fbdata(&ALL_MOTOR.left_front.DATA, rx_data);
                break;
            case 0x11 + LEG_LB:
                dm4310_fbdata(&ALL_MOTOR.left_back.DATA, rx_data);
                break;
            case 0x11 + LEG_RF:
                dm4310_fbdata(&ALL_MOTOR.right_front.DATA, rx_data);
                break;
            case 0x11 + LEG_RB:
                dm4310_fbdata(&ALL_MOTOR.right_back.DATA, rx_data);
                break;
        }
			
	}
	if (hcan == &hcan2)		
	{
		//CAN2
		switch (can_rx.StdId)
		{
            case LEG_WL:
                RUI_F_MOTOR_CAN_RX_3508RM(&ALL_MOTOR.left_wheel.DATA, rx_data);
                break;
            case LEG_WR:
                RUI_F_MOTOR_CAN_RX_3508RM(&ALL_MOTOR.right_wheel.DATA, rx_data);
                break;
        }
	}
}

void WHW_MCP2515_Callback(SPI_HandleTypeDef *hspi, uCAN_MSG *rxMessage)
{
    if (hspi->Instance == SPI2) {
        uint8_t CAN_SPI_2_Data[8];
        memcpy(CAN_SPI_2_Data, &rxMessage->array[6], 8);
        switch (rxMessage->frame.id)
        {
            case 0x201:

                break;
        }
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == SPI_CAN_2_EXIT_Pin) {
        uint8_t temp = WHW_CANSPI_Receive(&hspi2, &rxMessage1);
        if(temp)
            WHW_MCP2515_Callback(&hspi2, &rxMessage1);
        WHW_MCP2515_IRQHandler(&hspi2);
    }
}

#define  BUFFER_SIZE_6  (255)
#define  BUFFER_SIZE_1  (20)
#define  BUFFER_SIZE_3  (37)
void BSP_UART_IRQHandler(UART_HandleTypeDef *huart)
{
    if(huart->Instance ==USART3)//遥控接收串口
    {
        if (RESET != __HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE))
        {
			uint16_t temp = 0;
            __HAL_UART_CLEAR_IDLEFLAG(&huart3);  //清除空闲中断标志（否则会一直不断进入中断）
			temp = huart3.Instance -> SR; // 清除SR状态寄存器
			temp = huart3.Instance -> DR; // 清除DR数据寄存器，用来清除中断
            // 下面进行空闲中断相关处理
            HAL_UART_DMAStop(&huart3);//暂时停止本次DMA传输，进行数据处理
            
            if(BUFFER_SIZE_3 - __HAL_DMA_GET_COUNTER(&hdma_usart3_rx) == 18)
                RUI_F_DUBS_Resovled(DBUS_RX_DATA, &WHW_V_DBUS);

            HAL_UART_Receive_DMA(&huart3, (uint8_t *)DBUS_RX_DATA,37);  //重启开始DMA传输
        }
        
    }

    if(huart->Instance ==USART6)//裁判系统串口
    {
		uint8_t data_length_6;
        if (RESET != __HAL_UART_GET_FLAG(&huart6, UART_FLAG_IDLE))
        {
            __HAL_UART_CLEAR_IDLEFLAG(&huart6);  //清除空闲中断标志（否则会一直不断进入中断）
            // 下面进行空闲中断相关处理
            HAL_UART_DMAStop(&huart6);//暂时停止本次DMA传输，进行数据处理
            
            data_length_6  = BUFFER_SIZE_6 - __HAL_DMA_GET_COUNTER(&hdma_usart6_rx);//计算接收到的数据长度
		    Read_Data_first(&ALL_RX , &User_data , data_length_6);//测试函数：待修改
		    memset((uint8_t*)ALL_RX.Data,0,data_length_6);//清零接收缓冲区

            HAL_UART_Receive_DMA(&huart6,(uint8_t *)ALL_RX.Data,255);  //重启开始DMA传输
        }
    }

    if(huart->Instance ==USART1)//调试串口
    {
		//数据处理
		uint8_t data_length_1;
        if (RESET != __HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE))
        {
            __HAL_UART_CLEAR_IDLEFLAG(&huart1);  //清除空闲中断标志（否则会一直不断进入中断）
            // 下面进行空闲中断相关处理
            HAL_UART_DMAStop(&huart1);//暂时停止本次DMA传输，进行数据处理
            
            data_length_1 = BUFFER_SIZE_1 - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);//计算接收到的数据长度
		    memset((uint8_t *)RX, 0, data_length_1);

            HAL_UART_Receive_DMA(&huart1, (uint8_t *)RX, 20);  //重启开始DMA传输
        }
    }
}

