#ifndef __MYMATH_H
#define __MYMATH_H

#include "main.h"

typedef struct 
{
    struct 
    {
        float last_sum;
        float sum_num;
    }Sum;
    struct 
    {
        float last_diff;
        float diff_num;
    }Diff;
    float dt;
} Discreteness_TypeDef;

void Discreteness_Init(Discreteness_TypeDef *object);
void Discreteness_Sum(Discreteness_TypeDef *object, float input, float dt);
void Discreteness_Diff(Discreteness_TypeDef *object, float input, float dt);

#endif // !__MYMATH_H
