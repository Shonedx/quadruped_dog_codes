#include "motor.h"
#include "pid.h"
#include "can.h"
#include "stm32f4xx.h"
#include "stdlib.h"
//启动电机相关
extern MotorData_t can1_motors[4]; //电机数据结构体数组
extern MotorData_t can2_motors[4];

extern MotorSpeedPid_t motor_speed_pid; //速度环pid
extern MotorPosPid_t motor_pos_pid; //位置环pid

extern u8 can1_tx_buffer[8];
extern u8 can2_tx_buffer[8];
MotorOutputAngle_t motor_output_angle; //电机角度输出 只是用来装数据

void motorCalAbsAngle(MotorData_t *motor, u16 T) //计算电机绝对角度 T表示一个周期的角度值
{
    u16 angle1, angle2;
    static u16 pos_old;
	u16 pos;
    u16 motor_error; //机械角度差值
    u16 delta_angle; //角度变化值

    pos = motor->mechanical_angle; // 机械角度
    motor_error = pos - pos_old;

    if (motor_error > 0)
    {
        angle1 = motor_error - T; // 反转自减
        angle2 = motor_error;
    }
    else
    {
        angle1 = motor_error + T; // 正转，自加一个周期的角度值 (360)
        angle2 = motor_error;
    }

    if (abs(angle1) < abs(angle2)) // 不管正反转，肯定是转的角度小的那个是真的
    {
       delta_angle = angle1;
    }
    else
    {
       delta_angle = angle2;
    }
    motor->absolute_angle +=delta_angle;
    pos_old = pos;
}

void setMaxSpeed(u16 max_out)//设置速度环最大输出
{
	for(u8 i=0;i<4;i++)
	{
		motor_speed_pid.can1_motors_pid->max_out =max_out;
        motor_speed_pid.can2_motors_pid->max_out =max_out;
	}
}
void setMaxPos(u16 max_out)//设置角度环最大输出
{
	for(u8 i=0;i<4;i++)
	{
        motor_pos_pid.can1_motors_pid->max_out =max_out;
        motor_pos_pid.can2_motors_pid->max_out =max_out;
	}
}

void calPosOutput(u8 index,u8 can_id) // 计算角度环输出
{
    if(can_id==0) //can1
    {
        setTarAngle(&can1_motors[index], motor_output_angle.can1_motors_angle[index]); //把运动学逆解算出来角度值设置为角度环目标值
        can1_motors[index].output_angle = posPidCalculate(&motor_pos_pid.can1_motors_pid[index],  &can1_motors[index]);//角度环输出
    }
    else if(can_id==1) //can2
    {
        setTarAngle(&can2_motors[index], motor_output_angle.can2_motors_angle[index]); //把运动学逆解算出来角度值设置为角度环目标值
        can2_motors[index].output_angle = posPidCalculate(&motor_pos_pid.can1_motors_pid[index],  &can2_motors[index]);//角度环输出
    }
}
void calSpeedOutput(u8 index,u8 can_id) // 计算速度环输出
{
    if(can_id==0) //can1
    {
        setTarSpeed(&can1_motors[index], can1_motors[index].target_speed); //设置速度环目标值
        can1_motors[index].output_speed = speedPidCalculate(&motor_speed_pid.can1_motors_pid[index], &can1_motors[index]); //速度环输出
    }
    else if(can_id==1) //can2
    {
        setTarSpeed(&can2_motors[index], can2_motors[index].target_speed); //设置速度环目标值
        can2_motors[index].output_speed = speedPidCalculate(&motor_speed_pid.can2_motors_pid[index], &can2_motors[index]); //速度环输出
    }
}

//设置角度环目标值当前值
void setTarAngle(MotorData_t *motor_data,float target_angle) //设置角度环相关参数，这里传入了角度环的目标角度
{
	motor_data->target_angle = target_angle;
}
void setTarSpeed(MotorData_t *motor_data,float target_speed) //设置速度环相关参数，这里传入了速度环的目标速度
{
    motor_data->target_speed = target_speed;
}
void setOutCurrent(MotorData_t *motor_data,float output_current) //设置电流环相关参数，这里传入了电流环的目标电流，即速度环的输出
{
    motor_data->output_current = output_current;
}

void loadDataToCanTxBuf(void)
{
//can1
    can1_tx_buffer[0] = ((short)(can1_motors[0].output_current)) >> 8; //把数据装入待发送的can数据缓存数组
    can1_tx_buffer[1] = ((short)(can1_motors[0].output_current)) & 0x00FF;
    can1_tx_buffer[2] = ((short)(can1_motors[1].output_current)) >> 8;
    can1_tx_buffer[3] = ((short)(can1_motors[1].output_current)) & 0x00FF;
    can1_tx_buffer[4] = ((short)(can1_motors[2].output_current)) >> 8;
    can1_tx_buffer[5] = ((short)(can1_motors[2].output_current)) & 0x00FF;
    can1_tx_buffer[6] = ((short)(can1_motors[3].output_current)) >> 8;
    can1_tx_buffer[7] = ((short)(can1_motors[3].output_current)) & 0x00FF;
//can2
    can2_tx_buffer[0] = ((short)(can2_motors[0].output_current)) >> 8; //把数据装入待发送的can数据缓存数组
    can2_tx_buffer[1] = ((short)(can2_motors[0].output_current)) & 0x00FF;
    can2_tx_buffer[2] = ((short)(can2_motors[1].output_current)) >> 8;
    can2_tx_buffer[3] = ((short)(can2_motors[1].output_current)) & 0x00FF;
    can2_tx_buffer[4] = ((short)(can2_motors[2].output_current)) >> 8;
    can2_tx_buffer[5] = ((short)(can2_motors[2].output_current)) & 0x00FF;
    can2_tx_buffer[6] = ((short)(can2_motors[3].output_current)) >> 8;
    can2_tx_buffer[7] = ((short)(can2_motors[3].output_current)) & 0x00FF;
}
void setZeroToCanBuf(void)
{
    for(u8 i=0;i<8;i++)
    {
        can1_tx_buffer[i] = 0; //清空can1发送缓存
        can2_tx_buffer[i] = 0; //清空can2发送缓存
    }
}
void txMsgToMotor(u8 can_id)
{
    if(can_id == 0) //发送到CAN1
    {
        txDataToCAN(can1_tx_buffer, 8, 0);
    }
    else if (can_id == 1) //发送到CAN2
    {
        txDataToCAN(can2_tx_buffer, 8, 1);
    }
}

void runMotor(void) //驱动电机
{
	for(u8 i=0;i<4;i++)
	{
		motorCalAbsAngle(&can1_motors[i], 360); //计算绝对角度值
		motorCalAbsAngle(&can2_motors[i], 360); //计算绝对角度值
        setTarAngle(&can1_motors[i], motor_output_angle.can1_motors_angle[i]); //把运动学逆解算出来角度值设置为角度环目标值
        setTarAngle(&can2_motors[i], motor_output_angle.can2_motors_angle[i]); //把运动学逆解算出来角度值设置为角度环目标值
        calPosOutput(i, 0); //计算can1的角度环输出
        calPosOutput(i, 1); //计算can2的角度环输出
        setTarSpeed(&can1_motors[i], can1_motors[i].output_angle); //设置can1的速度环目标值
        setTarSpeed(&can2_motors[i], can2_motors[i].output_angle); //设置can2的速度环目标值
        calSpeedOutput(i, 0); //计算can1的速度环输出
        calSpeedOutput(i, 1); //计算can2的速度环输出
        setOutCurrent(&can1_motors[i], can1_motors[i].output_speed); //设置can1的电流输出值
        setOutCurrent(&can2_motors[i], can2_motors[i].output_speed); //设置can2的电流输出值
	}
    
    loadDataToCanTxBuf(); //把数据装入待发送的can数据缓存数组

	txMsgToMotor(0);//发送数据到can1的电机
    txMsgToMotor(1); //发送数据到can2的电机
}
void resetAbsoluteAngle(void)
{
	for(u8 i=0;i<4;i++)
	{
		can1_motors[i].absolute_angle=0;
		can2_motors[i].absolute_angle=0;
	}
}
