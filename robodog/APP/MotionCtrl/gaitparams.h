#ifndef _GaitParams_H
#define _GaitParams_H
#include "stdint.h"
#include "stm32f4xx.h"
typedef struct GaitParams
{
    float up_amp; //Amplitude 上升幅度
    float down_amp; //下降高度
    float stance_height; //站立高度
    float step_length;  //迈腿步长
    float freq; //频率
    float swing_percent; //摆动期占比
    float gait_offset;  //各腿的相位
    u8 leg_index; //腿序号
    float x_offset; //x初始值
} GaitParams_t;

extern GaitParams_t gait_params[][4]; //四种步态的参数

void setStandHeight(GaitParams_t *gait_params,u8 tar_height);
void setStepLength(GaitParams_t *gait_params);
void setUpAmp(GaitParams_t *gait_params);

#endif 


