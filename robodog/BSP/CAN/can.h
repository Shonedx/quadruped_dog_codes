#ifndef __CAN_H
#define __CAN_H

#define  CAN_ALL_ID  0x200 //两个can都用一样的id 且电调只用1，2，3，4号

#include "stm32f4xx.h"

typedef struct MotorData
{
    //从电机获取的数据 即当前值 current
    u16 current;  // 电流
    u16 mechanical_angle; //机械角度
    u16 speed;  // 速度
    u16 temperature; // 温度  
    u16 absolute_angle; //绝对角度
    //要设置给电机的数据
    u16 target_speed; //目标速度
    u16 target_angle; //目标角度
    //pid各环输出
    u16 output_angle; //角度环输出
    u16 output_speed; //速度环输出

    u16 output_current; //输出给电机的电流

} MotorData_t;


uint8_t canInit(void);
void calDataFromCAN(MotorData_t *motor_data,u8 *data); //解析can发来的数据
u8 rxDataFromCAN(u8 *rx_msg,u8 can_id);
u8 txDataToCAN(u8* tx_msg,u8 len,u8 can_id);




#endif /* __CAN_H */