#ifndef __MYMATH_H
#define __MYMATH_H

#include "main.h"
#include "can_bsp.h"

typedef struct 
{
    float last_sum;
    float sum_num;
    float last_diff;
    float diff_num;
    float dt;
} Discreteness_TypeDef;

void Discreteness_Init(Discreteness_TypeDef *object);
float Discreteness_Sum(Discreteness_TypeDef *object, float input, float dt);
float Discreteness_Diff(Discreteness_TypeDef *object, float input, float dt);
void DJI_Torque_Control(hcan_t* hcan, uint16_t stdid, float t1, float t2, float t3, float t4);

#endif // !__MYMATH_H
