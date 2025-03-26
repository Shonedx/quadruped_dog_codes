#include "Allheaderfile.h"
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
	


	/******************/
	int Jump_State=1;
	double jump_time=2; //��Ծ����
	double jump_freq=0.0020; //��Ծ�ٶ�
	double jump_angle=31.0f; //beta��������������ص㵽��˵���������ֱ����ļн�
	const float prep_time = 0.8f; // ׼��ʱ�� [s]		
    const float launch_time = 0.6f ; // ��Ծʱ�� [s]		
    const float fall_time = 0.8f; //���ʱ��[s]
	const float stance_height = 10.5; // ��Ծ֮ǰ�ȵĸ߶� [cm] 
    const float lauch_extension = 29; // ��Ծ�쳤ʱ�������ȳ� [cm]
	const float fall_height=16;	// ��غ��ȵĸ߶� [cm] 
	/******************/

	extern State currentstate;
	
	Leg legs[4];
//��̬���	
 /******************************************************************************************************/
void SinTrajectory(double t, GaitParams gaitparams)
{
	double x, z, gp;
	static double p = 0;
	static double prev_t = 0;
	p += gaitparams.freq * (t - prev_t);
	prev_t = t;
	gp = fmod(p + gaitparams.gaitoffset, 1.0);
	if (gp <= gaitparams.swingpercent) {
		x = (gp / gaitparams.swingpercent) * gaitparams.steplength - gaitparams.steplength / 2.0+ gaitparams.x_offset;
		z = -gaitparams.Up_Amp * sin(PI * gp / gaitparams.swingpercent) + gaitparams.stanceheight;
	}
	else {
		float percentBack = (gp - gaitparams.swingpercent) / (1.0 - gaitparams.swingpercent);
		x = -percentBack * gaitparams.steplength + gaitparams.steplength / 2.0 + gaitparams.x_offset;
		z = gaitparams.Down_Amp * sin(PI * percentBack) + gaitparams.stanceheight;
	}
	legs[gaitparams.i].x = x;
	legs[gaitparams.i].z = z;

} 
//void Translate(double t, GaitParams gaitparams)
//{
//	double x, z, gp;
//	static double p = 0;
//	static double prev_t = 0;
//	p += gaitparams.freq * (t - prev_t);
//	prev_t = t;
//	gp = fmod(p + gaitparams.gaitoffset, 1.0);
//	if (gp <= gaitparams.swingpercent) {
//		
//		z = -gaitparams.Up_Amp * sin(PI * gp / gaitparams.swingpercent) + gaitparams.stanceheight;
//	}
//	else {
//		float percentBack = (gp - gaitparams.swingpercent) / (1.0 - gaitparams.swingpercent);
//		
//		z = gaitparams.Down_Amp * sin(PI * percentBack) + gaitparams.stanceheight;
//	}
//	legs[gaitparams.i].x = 0;
//	legs[gaitparams.i].z = z;

//}
 void Gait(void)
{
	double t = 10*HAL_GetTick()/1000.0f;

	switch (currentstate)
	{
	case Idle:  // ԭ��̤��
		ChangeTheGainOfPID_KP_KI_KD(4.5,0.3,1.81,4.5,0.3,1.81);
		for (int i=0 ; i < 4; i++)
		{
			SinTrajectory(t, gaitparams[0][i]);
		}
		break;
	case Forward:
		ChangeTheGainOfPID_KP_KI_KD(4.1,0.3,1.81,4.1,0.3,1.81);
		for (int i = 0; i < 4; i++)
		{
			SinTrajectory(t, gaitparams[1][i]);
		}
		break;
	case Back:
		ChangeTheGainOfPID_KP_KI_KD(4.5,0.3,1.81,4.5,0.3,1.81);
		for (int i = 0; i < 4; i++)
		{
			SinTrajectory(t, gaitparams[2][i]);
		}

		break;
	case Turn_Left:
		ChangeTheGainOfPID_KP_KI_KD(4.5,0.3,1.81,4.5,0.3,1.81);
		for (int i = 0; i < 4; i++)
		{
			SinTrajectory(t, gaitparams[3][i]);
		}

		break;
	case Turn_Right:
		ChangeTheGainOfPID_KP_KI_KD(4.5,0.3,1.81,4.5,0.3,1.81);
		for (int i = 0; i < 4; i++)
		{
			SinTrajectory(t, gaitparams[4][i]);
		}

		break;
	case Jump:
		ChangeTheGainOfPID_KP_KI_KD(4.5,0.3,1.81,4.5,0.3,1.81);
		if(Jump_OK==1)
		{
			if(Jump_Start==1)
			{
				Start_Jump();
				Jump_Start=0;
			}
			Execute_Jump();
		}
		break;
	case Stop:
		ChangeTheGainOfPID_KP_KI_KD(4.5,0.3,1.81,4.5,0.3,1.81);
		for (int i = 0; i < 4; i++)
		{
			SinTrajectory(t, gaitparams[5][i]);
		}
		Jump_Start=1;
		Jump_OK = 1;

		break;
	
	case Translate_Left:
		ChangeTheGainOfPID_KP_KI_KD(3.8,0.3,1.81,3.8,0.3,1.81);
		for (int i = 0; i < 4; i++)
		{
			SinTrajectory(t, gaitparams[6][i]);
		}

		break;
		
	case Translate_Right:
		ChangeTheGainOfPID_KP_KI_KD(3.8,0.3,1.81,3.8,0.3,1.81);
		for (int i = 0; i < 4; i++)
		{
			SinTrajectory(t, gaitparams[7][i]);
		}

		break;
	}
	
	// ת�������˶�ѧ�ĽǶ�
	CartesianToTheta_Cycloid_All_Legs();
	// �����Ȳ��˶�
	Moveleg();
}


//�˶����
/******************************************************************************************************/
// ��⺯��
void CartesianToTheta_Cycloid(Leg *leg)
{
    leg->L = sqrt(leg->x * leg->x + leg->z * leg->z);
	if(leg->L<10) leg->L=10;
	if(leg->L>30) leg->L=30;
    leg->psai1 = asin(leg->x / leg->L);
    leg->fai1 = acos((leg->L * leg->L + L1 * L1 - L2 * L2) / (2 * L1 * leg->L));
    leg->theta2 = 180.0f * (leg->fai1 - leg->psai1) / PI - 90.0f;
    leg->theta1 = 180.0f * (leg->fai1 + leg->psai1) / PI - 90.0f;
}
////���������
void CartesianToTheta_Cycloid_All_Legs(void)
{
	for(int i=0;i<4;i++)
	{
		CartesianToTheta_Cycloid(&legs[i]);
	}
}
//������ذڶ���
void Angle_Setting_Cycloid(int LegID)  // Moveleg �ﱻ����
{ 
	switch (LegID)
	{
		case 0:
			Leg_angle.motorangle1 = -legs[0].theta1 * Gaito;  
			Leg_angle.motorangle2 = -legs[0].theta2 * Gaito;
		break;
		case 1:
			Leg_angle.motorangle3 =	legs[1].theta2 * Gaito;  
			Leg_angle.motorangle4 =	legs[1].theta1 * Gaito;
		break;
		case 2:
			Leg_angle.motorangle5 = -legs[2].theta1 * Gaito;
			Leg_angle.motorangle6 =	-legs[2].theta2 * Gaito;
		break;
		case 3:
			Leg_angle.motorangle7 =	legs[3].theta2 * Gaito;  
			Leg_angle.motorangle8 =	legs[3].theta1 * Gaito;
		break;
	}
}
// �ƶ��Ⱥ���
void Moveleg(void)
{
    // ����˵����
    // direction: �����־
    // Ϊÿ�������ýǶ�
    // Legid �� 0 �� 3 �ֱ����������
    Angle_Setting_Cycloid(0);  // ���õ� 0 ���ȵĽǶ�
    Angle_Setting_Cycloid(1);  // ���õ� 1 ���ȵĽǶ�
    Angle_Setting_Cycloid(2);  // ���õ� 2 ���ȵĽǶ�
    Angle_Setting_Cycloid(3);  // ���õ� 3 ���ȵĽǶ�
}


//����������
/******************************************************************************************************/

//Ϊÿ�����趨�Ƕ�
void AllLeg_Set_angle(int target_angle, int offset)
{
    // ID1
    Motor_Angle_Cal_1(360); // �õ����ԽǶ�
	Target_Pos_Setting(&pidmsg.M3508_STAND_ID1, -target_angle + offset);
	//Target_Pos_Setting(&pidmsg.M3508_STAND_ID1, -500); //����վ���Ƕ���
    motor_3508.ID1.angle_out = PID_Cal_STAND(&pidmsg.M3508_STAND_ID1,  motor_3508.ID1.POS_ABS);
    motor_3508.ID1.target_speed = motor_3508.ID1.angle_out; // �ǶȻ������Ϊ�ٶȻ�����
    motor_3508.ID1.current_output = PID_Calc(&pidmsg.M3508_SPEED_ID1,&motor_3508.ID1 );
    canbuf[0] = ((short)(motor_3508.ID1.current_output)) >> 8;
    canbuf[1] = ((short)(motor_3508.ID1.current_output)) & 0x00FF;

    // ID2
    Motor_Angle_Cal_2(360); // �õ����ԽǶ�
    Target_Pos_Setting(&pidmsg.M3508_STAND_ID2, target_angle + offset);
	//Target_Pos_Setting(&pidmsg.M3508_STAND_ID2, -500);
    motor_3508.ID2.angle_out = PID_Cal_STAND(&pidmsg.M3508_STAND_ID2,  motor_3508.ID2.POS_ABS);
    motor_3508.ID2.target_speed = motor_3508.ID2.angle_out; // �ǶȻ������Ϊ�ٶȻ�����
    motor_3508.ID2.current_output = PID_Calc( &pidmsg.M3508_SPEED_ID2,&motor_3508.ID2);
    canbuf[2] = ((short)(motor_3508.ID2.current_output)) >> 8;
    canbuf[3] = ((short)(motor_3508.ID2.current_output)) & 0x00FF;

    // ID3
    Motor_Angle_Cal_3(360); // �õ����ԽǶ�
    Target_Pos_Setting(&pidmsg.M3508_STAND_ID3, -target_angle - offset);
	//Target_Pos_Setting(&pidmsg.M3508_STAND_ID3,153);
    motor_3508.ID3.angle_out = PID_Cal_STAND(&pidmsg.M3508_STAND_ID3,  motor_3508.ID3.POS_ABS);
    motor_3508.ID3.target_speed = motor_3508.ID3.angle_out; // �ǶȻ������Ϊ�ٶȻ�����
    motor_3508.ID3.current_output = PID_Calc( &pidmsg.M3508_SPEED_ID3,&motor_3508.ID3);
    canbuf[4] = ((short)(motor_3508.ID3.current_output)) >> 8;
    canbuf[5] = ((short)(motor_3508.ID3.current_output)) & 0x00FF;

    // ID4
    Motor_Angle_Cal_4(360); // �õ����ԽǶ�
    Target_Pos_Setting(&pidmsg.M3508_STAND_ID4, target_angle - offset);
    //Target_Pos_Setting(&pidmsg.M3508_STAND_ID4, 2200);
    motor_3508.ID4.angle_out = PID_Cal_STAND(&pidmsg.M3508_STAND_ID4,motor_3508.ID4.POS_ABS);
    motor_3508.ID4.target_speed = motor_3508.ID4.angle_out; // �ǶȻ������Ϊ�ٶȻ�����
    motor_3508.ID4.current_output = PID_Calc(&pidmsg.M3508_SPEED_ID4,&motor_3508.ID4 ); 
    canbuf[6] = ((short)(motor_3508.ID4.current_output)) >> 8;
    canbuf[7] = ((short)(motor_3508.ID4.current_output)) & 0x00FF;

    // ����CAN1��Ϣ
    CAN1_Send_Msg(canbuf, 8, 0);

    // ID5
    Motor_Angle_Cal_5(360); // �õ����ԽǶ�
    Target_Pos_Setting(&pidmsg.M3508_STAND_ID5, target_angle + offset);
	//Target_Pos_Setting(&pidmsg.M3508_STAND_ID5, -1152);
    motor_3508.ID5.angle_out = PID_Cal_STAND(&pidmsg.M3508_STAND_ID5, motor_3508.ID5.POS_ABS);
    motor_3508.ID5.target_speed = motor_3508.ID5.angle_out; // �ǶȻ������Ϊ�ٶȻ�����
    motor_3508.ID5.current_output = PID_Calc(&pidmsg.M3508_SPEED_ID5,&motor_3508.ID5 );
    canbuf2[0] = ((short)(motor_3508.ID5.current_output)) >> 8;
    canbuf2[1] = ((short)(motor_3508.ID5.current_output)) & 0x00FF;

    // ID6
    Motor_Angle_Cal_6(360); // �õ����ԽǶ�
    Target_Pos_Setting(&pidmsg.M3508_STAND_ID6, -target_angle + offset);
	//Target_Pos_Setting(&pidmsg.M3508_STAND_ID6, -1152);
    motor_3508.ID6.angle_out = PID_Cal_STAND(&pidmsg.M3508_STAND_ID6,  motor_3508.ID6.POS_ABS);
    motor_3508.ID6.target_speed = motor_3508.ID6.angle_out; // �ǶȻ������Ϊ�ٶȻ�����
    motor_3508.ID6.current_output = PID_Calc( &pidmsg.M3508_SPEED_ID6,&motor_3508.ID6);
    canbuf2[2] = ((short)(motor_3508.ID6.current_output)) >> 8;
    canbuf2[3] = ((short)(motor_3508.ID6.current_output)) & 0x00FF;

    // ID7
    Motor_Angle_Cal_7(360); // �õ����ԽǶ�
    Target_Pos_Setting(&pidmsg.M3508_STAND_ID7, target_angle - offset);
	//Target_Pos_Setting(&pidmsg.M3508_STAND_ID7, 2200);
    motor_3508.ID7.angle_out = PID_Cal_STAND(&pidmsg.M3508_STAND_ID7,  motor_3508.ID7.POS_ABS);
    motor_3508.ID7.target_speed = motor_3508.ID7.angle_out; // �ǶȻ������Ϊ�ٶȻ�����
    motor_3508.ID7.current_output = PID_Calc( &pidmsg.M3508_SPEED_ID7,&motor_3508.ID7);
    canbuf2[4] = ((short)(motor_3508.ID7.current_output)) >> 8;
    canbuf2[5] = ((short)(motor_3508.ID7.current_output)) & 0x00FF;

    // ID8
    Motor_Angle_Cal_8(360); // �õ����ԽǶ�
   Target_Pos_Setting(&pidmsg.M3508_STAND_ID8, -target_angle - offset);
   //Target_Pos_Setting(&pidmsg.M3508_STAND_ID8, 153);
    motor_3508.ID8.angle_out = PID_Cal_STAND(&pidmsg.M3508_STAND_ID8,  motor_3508.ID8.POS_ABS);
    motor_3508.ID8.target_speed = motor_3508.ID8.angle_out; // �ǶȻ������Ϊ�ٶȻ�����
    motor_3508.ID8.current_output = PID_Calc(&pidmsg.M3508_SPEED_ID8,&motor_3508.ID8 );
    canbuf2[6] = ((short)(motor_3508.ID8.current_output)) >> 8;
    canbuf2[7] = ((short)(motor_3508.ID8.current_output)) & 0x00FF;

    // ����CAN2��Ϣ
    CAN2_Send_Msg(canbuf2, 8);
}

void Motor_Auto_Run(void) //�������
{
    // ID1
    Motor_Angle_Cal_1(360); // �õ����ԽǶ�
    Target_Pos_Setting(&pidmsg.M3508_POS_ID1, Leg_angle.motorangle1);
    motor_3508.ID1.angle_out = PID_Cal_POSITION(&pidmsg.M3508_POS_ID1, motor_3508.ID1.POS_ABS);
    motor_3508.ID1.target_speed = motor_3508.ID1.angle_out; // �ǶȻ������Ϊ�ٶȻ�����
    motor_3508.ID1.current_output = PID_Calc(&pidmsg.M3508_SPEED_ID1, &motor_3508.ID1);
    canbuf[0] = ((short)(motor_3508.ID1.current_output)) >> 8;
    canbuf[1] = ((short)(motor_3508.ID1.current_output)) & 0x00FF;

    // ID2
    Motor_Angle_Cal_2(360); // �õ����ԽǶ�
    Target_Pos_Setting(&pidmsg.M3508_POS_ID2, Leg_angle.motorangle2);
    motor_3508.ID2.angle_out = PID_Cal_POSITION(&pidmsg.M3508_POS_ID2, motor_3508.ID2.POS_ABS);
    motor_3508.ID2.target_speed = motor_3508.ID2.angle_out; // �ǶȻ������Ϊ�ٶȻ�����
    motor_3508.ID2.current_output = PID_Calc(&pidmsg.M3508_SPEED_ID2, &motor_3508.ID2);
    canbuf[2] = ((short)(motor_3508.ID2.current_output)) >> 8;
    canbuf[3] = ((short)(motor_3508.ID2.current_output)) & 0x00FF;

    // ID3
    Motor_Angle_Cal_3(360); // �õ����ԽǶ�
    Target_Pos_Setting(&pidmsg.M3508_POS_ID3, Leg_angle.motorangle3);
    motor_3508.ID3.angle_out = PID_Cal_POSITION(&pidmsg.M3508_POS_ID3, motor_3508.ID3.POS_ABS);
    motor_3508.ID3.target_speed = motor_3508.ID3.angle_out; // �ǶȻ������Ϊ�ٶȻ�����
    motor_3508.ID3.current_output = PID_Calc(&pidmsg.M3508_SPEED_ID3, &motor_3508.ID3);
    canbuf[4] = ((short)(motor_3508.ID3.current_output)) >> 8;
    canbuf[5] = ((short)(motor_3508.ID3.current_output)) & 0x00FF;

    // ID4
    Motor_Angle_Cal_4(360); // �õ����ԽǶ�
    Target_Pos_Setting(&pidmsg.M3508_POS_ID4, Leg_angle.motorangle4);
    motor_3508.ID4.angle_out = PID_Cal_POSITION(&pidmsg.M3508_POS_ID4, motor_3508.ID4.POS_ABS);
    motor_3508.ID4.target_speed = motor_3508.ID4.angle_out; // �ǶȻ������Ϊ�ٶȻ�����
    motor_3508.ID4.current_output = PID_Calc(&pidmsg.M3508_SPEED_ID4, &motor_3508.ID4);
    canbuf[6] = ((short)(motor_3508.ID4.current_output)) >> 8;
    canbuf[7] = ((short)(motor_3508.ID4.current_output)) & 0x00FF;

    // ����CAN1��Ϣ
    CAN1_Send_Msg(canbuf, 8, 0);

    // ID5
    Motor_Angle_Cal_5(360); // �õ����ԽǶ�
    Target_Pos_Setting(&pidmsg.M3508_POS_ID5, Leg_angle.motorangle5);
    motor_3508.ID5.angle_out = PID_Cal_POSITION(&pidmsg.M3508_POS_ID5, motor_3508.ID5.POS_ABS);
    motor_3508.ID5.target_speed = motor_3508.ID5.angle_out; // �ǶȻ������Ϊ�ٶȻ�����
    motor_3508.ID5.current_output = PID_Calc(&pidmsg.M3508_SPEED_ID5, &motor_3508.ID5);
    canbuf2[0] = ((short)(motor_3508.ID5.current_output)) >> 8;
    canbuf2[1] = ((short)(motor_3508.ID5.current_output)) & 0x00FF;

    // ID6
    Motor_Angle_Cal_6(360); // �õ����ԽǶ�
    Target_Pos_Setting(&pidmsg.M3508_POS_ID6, Leg_angle.motorangle6);
    motor_3508.ID6.angle_out = PID_Cal_POSITION(&pidmsg.M3508_POS_ID6, motor_3508.ID6.POS_ABS);
    motor_3508.ID6.target_speed = motor_3508.ID6.angle_out; // �ǶȻ������Ϊ�ٶȻ�����
    motor_3508.ID6.current_output = PID_Calc(&pidmsg.M3508_SPEED_ID6, &motor_3508.ID6);
    canbuf2[2] = ((short)(motor_3508.ID6.current_output)) >> 8;
    canbuf2[3] = ((short)(motor_3508.ID6.current_output)) & 0x00FF;

    // ID7
    Motor_Angle_Cal_7(360); // �õ����ԽǶ�
    Target_Pos_Setting(&pidmsg.M3508_POS_ID7, Leg_angle.motorangle7);
    motor_3508.ID7.angle_out = PID_Cal_POSITION(&pidmsg.M3508_POS_ID7, motor_3508.ID7.POS_ABS);
    motor_3508.ID7.target_speed = motor_3508.ID7.angle_out; // �ǶȻ������Ϊ�ٶȻ�����
    motor_3508.ID7.current_output = PID_Calc(&pidmsg.M3508_SPEED_ID7, &motor_3508.ID7);
    canbuf2[4] = ((short)(motor_3508.ID7.current_output)) >> 8;
    canbuf2[5] = ((short)(motor_3508.ID7.current_output)) & 0x00FF;

    // ID8
    Motor_Angle_Cal_8(360); // �õ����ԽǶ�
    Target_Pos_Setting(&pidmsg.M3508_POS_ID8, Leg_angle.motorangle8);
    motor_3508.ID8.angle_out = PID_Cal_POSITION(&pidmsg.M3508_POS_ID8, motor_3508.ID8.POS_ABS);
    motor_3508.ID8.target_speed = motor_3508.ID8.angle_out; // �ǶȻ������Ϊ�ٶȻ�����
    motor_3508.ID8.current_output = PID_Calc(&pidmsg.M3508_SPEED_ID8, &motor_3508.ID8);
    canbuf2[6] = ((short)(motor_3508.ID8.current_output)) >> 8;
    canbuf2[7] = ((short)(motor_3508.ID8.current_output)) & 0x00FF;

    // ����CAN2��Ϣ
    CAN2_Send_Msg(canbuf2, 8);

    flag_full = 0;
}






