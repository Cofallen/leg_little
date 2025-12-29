#include "mymath.h"

void Discreteness_Init(Discreteness_TypeDef *object)
{
    object->Sum.last_sum = 0;
    object->Sum.sum_num = 0;
    object->Diff.last_diff = 0;
    object->Diff.diff_num = 0;
    object->dt = 0;
}

void Discreteness_Sum(Discreteness_TypeDef *object, float input, float dt)
{
    object->dt = dt;
    object->Sum.sum_num += input * object->dt;
    object->Sum.last_sum = object->Sum.sum_num;
}


void Discreteness_Diff(Discreteness_TypeDef *object, float input, float dt)
{
    object->dt = dt;
    object->Diff.diff_num = (input - object->Diff.last_diff) / object->dt;
    object->Diff.last_diff = input;
}