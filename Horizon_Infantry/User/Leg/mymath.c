#include "mymath.h"

void Discreteness_Init(Discreteness_TypeDef *object)
{
    object->last_sum = 0;
    object->sum_num = 0;
    object->last_diff = 0;
    object->diff_num = 0;
    object->dt = 0;
}

float Discreteness_Sum(Discreteness_TypeDef *object, float input, float dt)
{
    object->dt = dt;
    object->sum_num += input * object->dt;
    object->last_sum = object->sum_num;
    return object->sum_num;
}


float Discreteness_Diff(Discreteness_TypeDef *object, float input, float dt)
{
    object->dt = dt;
    object->diff_num = (input - object->last_diff) / object->dt;
    object->last_diff = input;
    return object->diff_num;
}