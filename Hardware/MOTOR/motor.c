#include "motor.h"
//启动电机相关
Motors motors;
//计算绝对角度处理

// 全局变量
static uint8_t cnt[8] = {1, 1, 1, 1, 1, 1, 1, 1};

void Motor_Absolute_Angle_Cal(Motor_Property *motor, float T, uint8_t index)
{
    float res1, res2;
    static float pos_old[8];
	float pos;
    int motor_error[2];

    if (cnt[index])
    {
        pos_old[index] = motor->current_angle;
        cnt[index] = 0;
    }

    pos = motor->current_angle;
    motor_error[0] = pos - pos_old[index];

    if (motor_error[0] > 0)
    {
        res1 = motor_error[0] - T; // 反转自减
        res2 = motor_error[0];
    }
    else
    {
        res1 = motor_error[0] + T; // 正转，自加一个周期的角度值 (360)
        res2 = motor_error[0];
    }

    if (abs(res1) < abs(res2)) // 不管正反转，肯定是转的角度小的那个是真的
    {
        motor_error[1] = res1;
    }
    else
    {
        motor_error[1] = res2;
    }

    motor->absolute_angle += motor_error[1];
    pos_old[index] = pos;
}


void Set_Max_Output_SL(int max_out)//设置速度环最大输出
{
	for(int i=0;i<8;i++)
	{
		motor_speed_loop_pid.ID[i].max_out =max_out;
	}
}
void Set_Max_Output_PL( int max_out)//设置角度环最大输出
{
	for(int i=0;i<8;i++)
	{
		motor_position_loop_pid.ID[i].max_out =max_out;
	}

}
void Set_Motor_Target_Angle(int i)
{
	
	Set_Angle_Loop_Parameters(&motors.ID[i], motor_final_output_angles.ID[i],motors.ID[i].absolute_angle);
	motors.ID[i].output_angle = PID_Calc(&motor_position_loop_pid.ID[i],  &motors.ID[i]);//角度环输出
}
void Set_Motor_Target_Speed(int i)
{
	motors.ID[i].target_speed = motors.ID[i].output_angle; // 角度环输出作为速度环输入
}

void Set_Motor_Output_Current(int i)
{
	motors.ID[i].output_current = PID_Calc(&motor_speed_loop_pid.ID[i], &motors.ID[i]); // 输出给电机的电流
}

void Load_Data_To_Canbuf(int i)
{
	switch(i)
	{
		case 0:
			canbuf[0] = ((short)(motors.ID[0].output_current)) >> 8; //把数据装入待发送的can数据缓存数组
			canbuf[1] = ((short)(motors.ID[0].output_current)) & 0x00FF;
			break;
		case 1:
			canbuf[2] = ((short)(motors.ID[1].output_current)) >> 8;
			canbuf[3] = ((short)(motors.ID[1].output_current)) & 0x00FF;
			break;
		case 2:
			canbuf[4] = ((short)(motors.ID[2].output_current)) >> 8;
			canbuf[5] = ((short)(motors.ID[2].output_current)) & 0x00FF;
			break;
		case 3:
			canbuf[6] = ((short)(motors.ID[3].output_current)) >> 8;
			canbuf[7] = ((short)(motors.ID[3].output_current)) & 0x00FF;
			break;
		case 4:
			canbuf2[0] = ((short)(motors.ID[4].output_current)) >> 8;
			canbuf2[1] = ((short)(motors.ID[4].output_current)) & 0x00FF;
			break;
		case 5:
			canbuf2[2] = ((short)(motors.ID[5].output_current)) >> 8;
			canbuf2[3] = ((short)(motors.ID[5].output_current)) & 0x00FF;
			break;
		case 6:
			canbuf2[4] = ((short)(motors.ID[6].output_current)) >> 8;
			canbuf2[5] = ((short)(motors.ID[6].output_current)) & 0x00FF;
			break;
		case 7:
			canbuf2[6] = ((short)(motors.ID[7].output_current)) >> 8;
			canbuf2[7] = ((short)(motors.ID[7].output_current)) & 0x00FF;
			break;

	}
}

void Can1_Send_Msg_to_Motor(void)
{
	// 发送CAN1消息
    CAN1_Send_Msg(canbuf, 8);
}

void Can2_Send_Msg_to_Motor(void)
{
	 // 发送CAN2消息
    CAN2_Send_Msg(canbuf2, 8);
}

void Motor_Auto_Run(void) //驱动电机
{

	for(int i=0;i<8;i++)
	{
		Motor_Absolute_Angle_Cal(&motors.ID[i], 360, i); //计算绝对角度值
		Set_Motor_Target_Angle(i);
		Set_Motor_Target_Speed(i);
		Set_Motor_Output_Current(i);
		Load_Data_To_Canbuf(i);
	}
	Can1_Send_Msg_to_Motor();
	Can2_Send_Msg_to_Motor();

}
