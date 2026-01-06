#ifndef __GET_K_H
#define __GET_K_H

#include "vmc.h"

extern float ChassisL_LQR_K[12];
extern float ChassisR_LQR_K[12];

extern float ChassisL_LQR_K_fall[12];
extern float ChassisR_LQR_K_fall[12];

extern float ChassisL_LQR_K_err[12];
extern float ChassisR_LQR_K_err[12];

extern float ChassisL_LQR_K_stand[12];
extern float ChassisR_LQR_K_stand[12];

typedef union
{
    struct __packed
    {
        uint8_t header;
        float LQR_K[12];
        uint8_t tail;
    } K_data;
    uint8_t data[50];
} GetLQR_K_t;

extern GetLQR_K_t LQR_send_data;

int8_t Chassis_Get_K(GetLQR_K_t *rx_data);

#endif // !__GET_K_H