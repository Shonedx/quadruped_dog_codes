#ifndef Jump_H
#define Jump_H

#include "stm32f4xx.h"
#include "ges_cal.h"
#include "../DEFINE/define_file.h"
typedef struct JumpParameter
{
    Rotate_Stretch_t bend_struct[4]; // 旋转和伸展结构体
    Rotate_Stretch_t lean_struct[4]; // 旋转和伸展结构体
    Rotate_Stretch_t bend_lean_struct[4]; //倾斜的时候同时缩腿
    Rotate_Stretch_t exe_jump_struct[4]; // 旋转和伸展结构体
    Rotate_Stretch_t rcv_jump_struct[4]; // 旋转和伸展结构体
    u8 Bend_Flag; //俯身函数完成标志
    u8 Lean_Flag; //倾斜函数完成标志
    u8 Jump_OK; // Jump OK flag
} JumpParameter_t;
extern JumpParameter_t jump1_struct;
extern JumpParameter_t jump2_struct;

void jumpCtrl(float t,JumpParameter_t *js);
void jumpInit(JumpParameter_t *js,byte index);

#endif

