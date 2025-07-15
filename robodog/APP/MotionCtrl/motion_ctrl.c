#include "motion_ctrl.h"
#include "pid.h"
#include "RC.h"
#include "can.h"
#include "motor.h"
#include "gaitparams.h"
#include "timer.h"
#include "math.h"
//  LF(0)--------------------RF(3)
//  motor1       |             motor3             
//    (id:1)     |               (id:3)
//  motor2       |             motor4
//    (id:2)     |                (id:4)
//               |
//               |
//               |
//  LB(1)--------------------RB(2)
//   motor5                     motor7
//     (id:1)                     (id:3)
//   motor6                    motor8
//     (id:2)                     (id4)
//
// 2023-3-16  Motor_id_Sign


MotionState_t current_motion_state=MS_NORMAL;
LegParameter_t legs_params[4]; 

extern MotorOutputAngle_t motor_output_angle; //电机角度输出 只是用来装数据
extern 	int start;
extern CtrlState_t ctrl_state;
extern u8 pre_height; // 预设高度
extern JumpState_t jump_state;
extern GaitParams_t gait_params[][4];
extern volatile uint64_t timer;

//步态相关	
 /******************************************************************************************************/

void sinTrajectory(float t, GaitParams_t gait_params) //正弦轨迹逆解
{
	float x, z, gp;
	volatile static float p = 0;
	volatile static float prev_t = 0;
	p += gait_params.freq * (t - prev_t);
	prev_t = t;
	gp = fmod(p + gait_params.gait_offset, 1);
    
    if(current_motion_state==MS_STOP)
    {
        p=0;
        prev_t = 0;
    }

	if (gp <= gait_params.swing_percent) {
		x = (gp / gait_params.swing_percent) * gait_params.step_length - gait_params.step_length / 2.0+ gait_params.x_offset;
		z = -gait_params.up_amp * sin(PI * gp / gait_params.swing_percent) + gait_params.stance_height;
	}
	else {
		float percentBack = (gp - gait_params.swing_percent) / (1.0 - gait_params.swing_percent);
		x = -percentBack * gait_params.step_length + gait_params.step_length / 2.0 + gait_params.x_offset;
		z = gait_params.down_amp * sin(PI * percentBack) + gait_params.stance_height;
	}
	legs_params[gait_params.leg_index].x = x;
	legs_params[gait_params.leg_index].z = z;
} 

//	CS_NONE,
//	CS_INIT,
//	CS_MAIN,
//	CS_PRE_JUMP,
//	CS_EXE_JUMP,
//	CS_HEIGHT,
//	CS_QUIT,

void motionStateCtrl(void)
{
	switch(ctrl_state)
	{
		case CS_NONE://初始状态 
			start=0;
			break;
		case CS_INIT://初始化机器狗
			start=0;
            resetAbsoluteAngle(); //设置电机初始位置
			break;
		case CS_MAIN://主控制
			rcMotionCtrl();
			start=1;
			break;
		case CS_PRE_JUMP:// 跳跃准备
			start=1;
			break;
		case CS_EXE_JUMP:// 跳跃执行
			start=1;
			break;
		case CS_HEIGHT:
			setStandHeight(gait_params[0],pre_height); //NORMAL
			setStandHeight(gait_params[1],pre_height);//STOP
			setStandHeight(gait_params[2],pre_height);//TRANS_LEFT
			setStandHeight(gait_params[3],pre_height);//TRANS_RIGHT
			setStepLength(gait_params[0]);
			setStepLength(gait_params[1]);
			setUpAmp(gait_params[0]);
			setUpAmp(gait_params[1]);
			setUpAmp(gait_params[2]);
			setUpAmp(gait_params[3]);
			start=1;
			break;
		case CS_QUIT:
			start=0;
			break;
		default:
			break;
	}
}
void gaitCtrl(float t)
{
	switch (current_motion_state)
	{
		if(ctrl_state!=CS_HEIGHT)
		{
			case MS_NORMAL:
				setMaxSpeed(8000);
				setMaxPos(8000);
				setSpeedPosPid(7.5,0.3,1.81,7.5,0.3,2.5);
				RC_StepLengthCtrl(gait_params[0]);
				for (int i=0 ; i < 4; i++)
				{
					sinTrajectory(t, gait_params[0][i]);
				}
				break;
			if(if_in_normal_range(pre_height, 14, 30)) //高度合适时
			{
				case MS_TRANSLATE_LEFT:
					setMaxSpeed(10000);
					setMaxPos(10000);
					setSpeedPosPid(7.5,0.3,1.81,7.5,0.3,2.5);
					for (int i = 0; i < 4; i++)
					{
						sinTrajectory(t, gait_params[2][i]);
					}
					break;
				
				case MS_TRANSLATE_RIGHT:
					setMaxSpeed(10000);
					setMaxPos(10000);
					setSpeedPosPid(7.5,0.3,1.81,7.5,0.3,2.5);
					for (int i = 0; i < 4; i++)
					{
						sinTrajectory(t, gait_params[3][i]);
					}
					break;
			}
		}
		case MS_STOP:
			setMaxSpeed(8000);
			setMaxPos(8000);
			setSpeedPosPid(7.5,0.3,1.81,7.5,0.3,2.5);
			for (int i = 0; i < 4; i++)
			{
                sinTrajectory(t, gait_params[1][i]); //调用此函数是为了清零函数中断static变量
				legs_params[gait_params[1][i].leg_index].x = 0;
				legs_params[gait_params[1][i].leg_index].z = gait_params[1][i].stance_height;
            }
			break;
	}
	jump_state=IDLE;
	// 转换到逆运动学的角度
	transXYToAngle_SUM();
	// 传输各腿电机角度
	setMotionOutAngle();
	runMotor();

}
//运动相关
/******************************************************************************************************/
// 逆解函数
void transXYToAngle(LegParameter_t *leg)
{
    leg->L = sqrt(leg->x * leg->x + leg->z * leg->z);
	if(leg->L<MinHeight) leg->L=MinHeight;
	if(leg->L>MaxHeight) leg->L=MaxHeight;
    leg->psai1 = asin(leg->x / leg->L);
    leg->fai1 = acos((leg->L * leg->L + L1 * L1 - L2 * L2) / (2 * L1 * leg->L));
    leg->theta2 = 180.0f * (leg->fai1 - leg->psai1) / PI - 90.0f;
    leg->theta1 = 180.0f * (leg->fai1 + leg->psai1) / PI - 90.0f;
}
////逆解所有腿
void transXYToAngle_SUM(void) 
{
	for(int i=0;i<4;i++)
	{
		transXYToAngle(&legs_params[i]);
	}
}
//设置相关摆动角
void setMotionOutAngle(void) 
{ 
    //某一侧的腿
    motor_output_angle.can1_motors_angle[0]=legs_params[0].theta1 * RotorRatio;  //这个函数把逆解函数逆解出的角度给到最最终要送至PID控制器进行计算的数组
    motor_output_angle.can1_motors_angle[1]=legs_params[0].theta2 * RotorRatio;
    motor_output_angle.can1_motors_angle[2]=-legs_params[2].theta2 * RotorRatio;
    motor_output_angle.can1_motors_angle[3]=-legs_params[2].theta1 * RotorRatio;
	//另一侧的腿
    motor_output_angle.can2_motors_angle[0]=legs_params[1].theta1 * RotorRatio;
    motor_output_angle.can2_motors_angle[1]=legs_params[1].theta2 * RotorRatio;
    motor_output_angle.can2_motors_angle[2]=-legs_params[3].theta2 * RotorRatio;
    motor_output_angle.can2_motors_angle[3]=-legs_params[3].theta1 * RotorRatio;
}
void rotateLegs(float tar_angle) //旋转腿部
{
	// for(int i=0;i<4;i++)
	// {
	// 	legs_params[i].psai1 += angle; //每次调用此函数，腿部的摆动角度都会增加angle
	// 	if(legs_params[i].psai1>360) legs_params[i].psai1 -= 360;
	// 	if(legs_params[i].psai1<0) legs_params[i].psai1 += 360;
	// }
	// transXYToAngle_SUM();
	// setMotionOutAngle();
	// runMotor();
}