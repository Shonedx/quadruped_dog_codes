# **������������ƴ���USER�ļ�����**
## **Main**
- main.cִ����Ҫ�߼�
- main.h �����һЩ����
- main_params.h�Ǵ��main�������ı������Ը������ļ����ñ���,�������Ͳ鿴
## **Ges_cal�ļ�**
1. GaitParams�����˲�̬�ĸ���״̬
    -����:
    ```cpp
    GaitParams gaitparams[][4] = {
    //	Up_Amp Down_Amp stanceheight steplength freq swingpercent gaitoffset i x_offset
    //	�������� �½��߶� վ���߶� ���Ȳ��� Ƶ�� �ڶ���ռ�� ���ȵ���λ ����� x��ʼֵ
    {//ԭ��̤����������״̬ 0
        { 4, 0.2, StandHeight, 0, Freq, 0.25, 0, 0,X_OFFSET},
        { 4, 0.2, StandHeight, 0, Freq, 0.25, 0.5, 1,X_OFFSET},
        { 4, 0.2, StandHeight, 0, Freq, 0.25, 0.5, 2,X_OFFSET},
        { 4, 0.2, StandHeight, 0, Freq, 0.25, 0, 3,X_OFFSET},
    },
    {//ǰ��״̬ 1
        { 5, 0.2, StandHeight, 16, Freq, 0.25, 0, 0,X_OFFSET},
        { 5, 0.2, StandHeight, 16, Freq, 0.25, 0.5, 1,X_OFFSET},
        { 5, 0.2, StandHeight, 16, Freq, 0.25, 0.5, 2,X_OFFSET},
        { 5, 0.2, StandHeight, 16, Freq, 0.25, 0, 3,X_OFFSET},
    },
    {//����״̬ 2
        { 5, 0.2, StandHeight, -16, Freq, 0.25, 0, 0,X_OFFSET},
        { 5, 0.2, StandHeight,-16, Freq, 0.25, 0.5, 1,X_OFFSET},
        { 5, 0.2, StandHeight, -16, Freq, 0.25, 0.5, 2,X_OFFSET},
        { 5, 0.2, StandHeight, -16, Freq, 0.25, 0, 3,X_OFFSET},
    },
    {//��ת��״̬ 3
        { 4, 0.2, StandHeight, -8, 3, 0.25, 0, 0,X_OFFSET},
        { 4, 0.2, StandHeight, 8, 3, 0.25, 0.5, 1,X_OFFSET},
        { 4, 0.2, StandHeight, -8, 3, 0.25, 0.5, 2,X_OFFSET},
        { 4, 0.2, StandHeight, 8, 3, 0.25, 0, 3,X_OFFSET},
    },
    {//��ת��״̬ 4
        { 4, 0.2, StandHeight, 8, 3, 0.25, 0, 0,X_OFFSET},
        { 4, 0.2, StandHeight, -8, 3, 0.25, 0.5, 1,X_OFFSET},
        { 4, 0.2, StandHeight, 8, 3, 0.25, 0.5, 2,X_OFFSET},
        { 4, 0.2, StandHeight, -8, 3, 0.25, 0, 3,X_OFFSET},
    },

    {//ֹͣ 5
        { 0, 0, StandHeight, 0, 0, 0, 0, 0,X_OFFSET},
        { 0, 0, StandHeight, 0, 0, 0, 0, 1,X_OFFSET},
        { 0, 0, StandHeight, 0, 0, 0, 0, 2,X_OFFSET},
        { 0, 0, StandHeight, 0, 0, 0, 0, 3,X_OFFSET},
    },
     //ʡ��.....
    ```
2. ges_cal �а���������Ҫ�Ĵ����߼�
    -����
    ```cpp
    void Gait(void)
    {
        double t = 10*HAL_GetTick()/1000.0f;

        switch (currentstate)
        {
        case Idle:  // ԭ��̤��
            
            for (int i=0 ; i < 4; i++)
            {
                SinTrajectory(t, gaitparams[0][i]);
            }
            break;
        case Forward:
            for (int i = 0; i < 4; i++)
            {
                SinTrajectory(t, gaitparams[1][i]);
            }
            break;
        case Back:
            for (int i = 0; i < 4; i++)
            {
                SinTrajectory(t, gaitparams[2][i]);
            }

            break;
        case Turn_Left:
            for (int i = 0; i < 4; i++)
            {
                SinTrajectory(t, gaitparams[3][i]);
            }

            break;
        case Turn_Right:
            for (int i = 0; i < 4; i++)
            {
                SinTrajectory(t, gaitparams[4][i]);
            }

            break;
        case Jump:
            if(Jump_OK==1)
            {
                Start_Jump();
                Execute_Jump();
            }

            break;
        case Stop:
            for (int i = 0; i < 4; i++)
            {
                SinTrajectory(t, gaitparams[5][i]);
            }

            Jump_OK = 0;

            break;
        }
        // ת�������˶�ѧ�ĽǶ�
        CartesianToTheta_Cycloid_All_Legs();
        // �����Ȳ��˶�
        Moveleg();
    }
    ```
3.  Gait������:
    ```cpp
        double t = 10*HAL_GetTick()/1000.0f;
    ```
    - ͨ��HAL_GetTick()��ȡϵͳ��ʵʱʱ������Ϊ�������˶����˶�����
    ```cpp
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
    ```
    - ͨ�������ҹ켣�滮���滮��������������ߣ��������е�һЩ��������ges_cal.c����������
    - ͨ��ʶ��currentstate״̬---����GaitParams.c�е� ```cpp State currentstate; ```ʵ���л���̬״̬��Ч��
    -���ͨ�� 
    ```cpp  
        // ת�������˶�ѧ�ĽǶ�
        CartesianToTheta_Cycloid_All_Legs();
        // �����Ȳ��˶�
        Moveleg();
    ```
    ������˹켣ͨ�����˶�ѧ���㴦�������Ƕȣ�Moveleg()�����Ƕ����õ������ȵĵ��
4. �����������
    - AllLeg_Set_angle()��Motor_Auto_Run()�ֱ����������õ���Ƕȵĺ���������ִ�е���˶��ĺ���
    - ��ȡһС�δ��� from Motor_Auto_Run()
    ```cpp
    //����һ�ι��ڵ��1�Ĵ��루�ܹ��˸������
     // ID1
    Motor_Angle_Cal_1(360); // �õ����ԽǶ�
    Target_Pos_Setting(&pidmsg.M3508_POS_ID1, Leg_angle.motorangle1);
    motor_3508.ID1.angle_out = PID_Cal_POSITION(&pidmsg.M3508_POS_ID1, motor_3508.ID1.POS_ABS);
    motor_3508.ID1.target_speed = motor_3508.ID1.angle_out; // �ǶȻ������Ϊ�ٶȻ�����
    motor_3508.ID1.current_output = PID_Calc(&pidmsg.M3508_SPEED_ID1, &motor_3508.ID1);
    canbuf[0] = ((short)(motor_3508.ID1.current_output)) >> 8;
    canbuf[1] = ((short)(motor_3508.ID1.current_output)) & 0x00FF;
    ```
5. jump��ش���
    - Start_Jump()������ȡ��ʼ��Ծ�ĳ�ʼʱ��
    ```cpp
    void Start_Jump(void)
    {
        start_time = HAL_GetTick();
    }
    ```
    - Reset_start_time()��������Ծ����ʱ���»�ȡ��һ����Ծ��ʼʱ��ʱ��(���Դ˴ﵽ������Ծ)
    ```cpp
    void Reset_start_time(void) 
    {
        start_time = HAL_GetTick();
    }
    ```
    - Execute_Jump()����ִ����Ծ�߼�
    ```cpp
    void Execute_Jump(void)
    {
        double t = 1.0*(HAL_GetTick()/1000.0f  - start_time/1000.0f);
    
        for (int i = 0; i < 4; i++) {
            if (t < 0.6) {

                double x = -2.08;
                double z = 11.81;
                legs[i].x = x;
                legs[i].z = z;
            //printf("waiting\n");
            }
            else if (t >= 0.6 && t < 1.0) {
                double x = -38.89;
                double z = 38.89;
                legs[i].x = x;
                legs[i].z = z;
                //printf("Jump!\n");

            }//&& t < 1.5
            else if (t >= 1.0 && t < 1.5) {

                double x = 0;
                double z = -17;
                legs[i].x = x;
                legs[i].z = z;
                //printf("fall!\n");
            }
            else {

                if (i == 3)
                {
                    Reset_start_time();
                Jump_OK = 0;	//�����Ծ����
                }
            }
        }

    }
    ```

## **����ִ���߼�**
- ��timer.c�а����˿���ִ�е��߼�
    - �����ڶ�ʱ��4���жϺ�������ִ��Gait()����ģ��
    ```cpp
    void TIM4_IRQHandler() //�������߼�
    {  
        if(TIM_GetITStatus(TIM4, TIM_IT_Update) !=RESET)
        {
            if(start==1)
            Gait(); //ִ�в�̬�߼�
            TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
        }
    ```
    - ���ڶ�ʱ��5�е��жϺ�������ִ�п���ģʽ��״̬ת��
    ```cpp
    void TIM5_IRQHandler(void) //״̬��
    {
        if(TIM_GetITStatus(TIM5,TIM_IT_Update)!= RESET)
        {		

        switch(ctrl_state)
        {
                case Initial_Ctrl://��ʼ״̬ 
                M3508_ALL_ZERO_SET();
                start=0;
                if_idle=0;
                break;
                case Start_Ctrl://��ʼ��������
                start=1;
                if_idle=0;
                Ctrl_Cmd();
                break;
                
                case Main_Ctrl://������
                if_idle=1;
                start=1;
                Ctrl_Cmd();
                break;
                
                case Stop_Ctrl:// ֹͣ
                if_idle=0;
                start=1;
                Ctrl_Cmd();
                break;
                
                case Jump_Ctrl:// ��Ծ
                if_idle=0;
                start=1;
                Jump_OK=1;
                Ctrl_Cmd();
                break;
        }
            if(start==1)
            {
                Motor_Auto_Run();
            }
            feed_dog();
        }
        TIM_ClearITPendingBit(TIM5,TIM_IT_Update); //����жϱ�־λ
    }
    ```
    - git����ģʽ��״̬����:
    ```cpp
    typedef enum 
    {
        Initial_Ctrl, //
        Start_Ctrl,
        Main_Ctrl,
        Stop_Ctrl,
        Jump_Ctrl,
    }Ctrl_State; //������״̬��
    ```
## **Զ�̿��ƴ���---Remote_Ctrl�ļ�**
1. RemoteControl_Init�а����˴�ң�����������ݵĴ������ﲻ���������(�һ�����ô�˽�ò���)
2. RC_Command����
    - Remote_Cmd()�����ı����ģ���״̬
    ```cpp  
    void Remote_Cmd(void)
    {
        if(LPS==2&&RPS==1)
        {
            ctrl_state=Initial_Ctrl;
        }
        else if(LPS==3&&RPS==1)
        {
            ctrl_state=Start_Ctrl;
        }
        else if(LPS==1&&RPS==1)
        {
            ctrl_state=Main_Ctrl;
        }
        else if(LPS==1&&RPS==3)
        {
            ctrl_state=Stop_Ctrl;
        }
        else if(LPS==3&&RPS==3)
        {
            ctrl_state=Jump_Ctrl;
        }
    //	else if(LPS==2&&RPS==3)
    //	{
    //		ctrl_state=JUMP_2;
    //	}
    }    
    ```
    - Ctrl_Cmd()����ͨ��ң����������ָ�����ı�currenstate���������˶���״̬(ǰ�������ˣ�����ת�ȵ�)
    ```cpp
    void Ctrl_Cmd(void) //���������
    {
        //ǰ��
        if (left_y >= 230 && abs(left_x) <= 330 && abs(right_y) <= 330 && abs(right_x) <= 330  ) 
        {	
            currentstate=Forward;
        }
        // ��ƽ��
        else if (right_x <= -100 && abs(right_y) <= 230  && abs(left_y) <= 330 && abs(left_x) <= 330 )
        {
            
            currentstate=Translate_Left;
        }
        // ��ƽ��
        else if (right_x >= 100  && abs(right_y) <= 230 && abs(left_y) <= 330 && abs(left_x) <= 330) 
        {
            
            currentstate=Translate_Right;
        }
        // ��ת
        else if (left_x <= -100 && abs(left_y) <= 230  && abs(right_y) <= 330 && abs(right_x) <= 330 ) 
        {
            
            currentstate=Turn_Left;
        }
        // ��ת
        else if (left_x >= 100  && abs(left_y) <= 230  && abs(right_y) <= 330 && abs(right_x) <= 330) 
        {
            
            currentstate=Turn_Right;
        }
        // ����
        else if (left_y <= -230 && abs(left_x) <= 330 && abs(right_y) <= 330 && abs(right_x) <= 330 ) 
        {	
        
            currentstate=Back;
        }
        else if (right_y >= 230 && abs(right_x) <= 330  && abs(left_y) <= 330 && abs(left_x) <= 330 ) 
        {	
            currentstate=Jump;
        }

    // ���� 
        else if 
            (
                abs(left_y) <= 330 
                && abs(left_x) <= 330
                &&abs(right_x) <= 330
                &&abs(right_y) <= 330 
            ) 
        {
            if(if_idle==1)
            currentstate=Idle;
            else if(if_idle==0)
            currentstate=Stop;
            TIM_Cmd(TIM4, ENABLE);
        }
        
    }
    ```
## **IMUģ��(IMU�ļ�)**
1. δ�����...