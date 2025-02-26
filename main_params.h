#ifndef __MAIN_PARAMS_H
#define __MAIN_PARAMS_H
#include "Allheaderfile.h"
#include "sys.h"

//注:该文件用于统一讲main中声明的变量放到头文件中用以给其他文件调用变量，方便管理和查看



extern 	int start; //初始化标识符
extern int feed; //喂狗
extern  int reset;	//重置标识符

extern	int left_x,left_y,right_x,right_y; //RemoteControl中声明的
extern 	int left_push_stick, right_push_stick; //左上，右上推杆

#endif
