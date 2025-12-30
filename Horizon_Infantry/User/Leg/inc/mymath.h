#ifndef __MYMATH_H
#define __MYMATH_H

#include "main.h"

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

#endif // !__MYMATH_H
