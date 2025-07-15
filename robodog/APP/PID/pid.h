#ifndef _PID_H
#define _PID_H
#include "stdint.h"
#include "stm32f4xx.h"
#include "can.h"
 enum PID_MODE //pid位置环和速度环的选择
{
    PID_POSITION = 0,
    PID_INCREMENTAL= 1
};

typedef struct PidPrameters
{
    u8 mode;           // 控制模式：PID_POSITION 或 PID_DELTA

    float Kp;               // 比例增益
    float Ki;               // 积分增益
    float Kd;               // 微分增益

    float max_out;          // 最大输出
    float max_iout;         // 最大积分输出

    float tar;              // 目标值
    float fdb;              // 反馈值

    float out;              // 总输出

    float Pout;             // 比例项输出
    float Iout;             // 积分项输出
    float Dout;             // 微分项输出

    float Dbuf[3];          // 微分项缓存

    float error[3];         // 误差缓存

    float dead_zone;         // 死区
} PidPrameter_t;

typedef struct MotorSpeedPid
{
	PidPrameter_t can1_motors_pid[4];
	PidPrameter_t can2_motors_pid[4];
}MotorSpeedPid_t;

typedef struct MotorPosPid
{
	PidPrameter_t can1_motors_pid[4];
	PidPrameter_t can2_motors_pid[4];
}MotorPosPid_t;

typedef struct ImuEulerAnglePid
{
	PidPrameter_t Yaw;
	PidPrameter_t Pitch;
	PidPrameter_t Roll;
}ImuEulerAnglePid_t;


/*************函数*******************/
void pidSpeedInit(MotorSpeedPid_t *pid) ;//速度环初始化
void pidPosInit(MotorPosPid_t *pid); //位置环初始化
void imuPidInit(ImuEulerAnglePid_t *pid) ;//欧拉角pid环初始化
void pidInit(void); //初始化所有pid
float pidCalculate(PidPrameter_t *pid,float tar,float fdb);// PID计算公式
float speedPidCalculate(PidPrameter_t *pid,MotorData_t *motor_data); // 计算速度环输出
float posPidCalculate(PidPrameter_t *pid,MotorData_t *motor_data) ;// 计算角度环输出
void setPidFactor(PidPrameter_t *pid,float kp,float kd,float ki) ;
void setPosPidFactor(float kp,float kd,float ki); //设置角度环pid系数
void setSpeedPidFactor(float kp,float kd,float ki) ;//设置速度环的pid系数
void setSpeedPosPid(float sp_kp,float sp_ki,float sp_kd,float pos_kp,float pos_ki,float pos_kd) ;//设置角度环和速度环的对应参数

#endif /* _PID_H */
