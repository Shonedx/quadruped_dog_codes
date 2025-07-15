#ifndef __MOTOR_H
#define __MOTOR_H
#include "can.h"
typedef struct MotorOutputAngle
{
	u16 can1_motors_angle[4];
	u16 can2_motors_angle[4];
}MotorOutputAngle_t;  	

extern MotorOutputAngle_t motor_output_angle; //电机角度输出 只是用来装数据


void motorCalAbsAngle(MotorData_t *motor, u16 T); //计算电机绝对角度 T表示一个周期的角度值
void setMaxSpeed(u16 max_out);//设置速度环最大输出
void setMaxPos(u16 max_out);//设置角度环最大输出
void calPosOutput(u8 index,u8 can_id) ;// 计算角度环输出
void calSpeedOutput(u8 index,u8 can_id) ;// 计算速度环输出
void setTarAngle(MotorData_t *motor_data,float target_angle); //设置角度环相关参数，这里传入了角度环的目标角度
void setTarSpeed(MotorData_t *motor_data,float target_speed); //设置速度环相关参数，这里传入了速度环的目标速度
void setOutCurrent(MotorData_t *motor_data,float output_current); //设置电流环相关参数，这里传入了电流环的目标电流，即速度环的输出
void loadDataToCanTxBuf(void);
void setZeroToCanBuf(void); //清空can发送缓存 常用于使电机停止运作
void txMsgToMotor(u8 can_id);
void runMotor(void); //驱动电机
void resetAbsoluteAngle(void);//设置电机初始绝对角度位置


#endif // __MOTOR_H


