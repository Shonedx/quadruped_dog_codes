# **四足机器狗控制代码USER文件部分**
## **Main**
- main.c执行主要逻辑
- main.h 存放了一些定义
- main_params.h是存放main中声明的变量用以给其他文件调用变量,方便管理和查看
## **Ges_cal文件**
1. GaitParams储存了步态的各个状态
    -如下:
    ```cpp
    GaitParams gaitparams[][4] = {
    //	Up_Amp Down_Amp stanceheight steplength freq swingpercent gaitoffset i x_offset
    //	上升幅度 下降高度 站立高度 迈腿步长 频率 摆动期占比 各腿的相位 腿序号 x初始值
    {//原地踏步（待机）状态 0
        { 4, 0.2, StandHeight, 0, Freq, 0.25, 0, 0,X_OFFSET},
        { 4, 0.2, StandHeight, 0, Freq, 0.25, 0.5, 1,X_OFFSET},
        { 4, 0.2, StandHeight, 0, Freq, 0.25, 0.5, 2,X_OFFSET},
        { 4, 0.2, StandHeight, 0, Freq, 0.25, 0, 3,X_OFFSET},
    },
    {//前进状态 1
        { 5, 0.2, StandHeight, 16, Freq, 0.25, 0, 0,X_OFFSET},
        { 5, 0.2, StandHeight, 16, Freq, 0.25, 0.5, 1,X_OFFSET},
        { 5, 0.2, StandHeight, 16, Freq, 0.25, 0.5, 2,X_OFFSET},
        { 5, 0.2, StandHeight, 16, Freq, 0.25, 0, 3,X_OFFSET},
    },
    {//后退状态 2
        { 5, 0.2, StandHeight, -16, Freq, 0.25, 0, 0,X_OFFSET},
        { 5, 0.2, StandHeight,-16, Freq, 0.25, 0.5, 1,X_OFFSET},
        { 5, 0.2, StandHeight, -16, Freq, 0.25, 0.5, 2,X_OFFSET},
        { 5, 0.2, StandHeight, -16, Freq, 0.25, 0, 3,X_OFFSET},
    },
    {//左转弯状态 3
        { 4, 0.2, StandHeight, -8, 3, 0.25, 0, 0,X_OFFSET},
        { 4, 0.2, StandHeight, 8, 3, 0.25, 0.5, 1,X_OFFSET},
        { 4, 0.2, StandHeight, -8, 3, 0.25, 0.5, 2,X_OFFSET},
        { 4, 0.2, StandHeight, 8, 3, 0.25, 0, 3,X_OFFSET},
    },
    {//右转弯状态 4
        { 4, 0.2, StandHeight, 8, 3, 0.25, 0, 0,X_OFFSET},
        { 4, 0.2, StandHeight, -8, 3, 0.25, 0.5, 1,X_OFFSET},
        { 4, 0.2, StandHeight, 8, 3, 0.25, 0.5, 2,X_OFFSET},
        { 4, 0.2, StandHeight, -8, 3, 0.25, 0, 3,X_OFFSET},
    },

    {//停止 5
        { 0, 0, StandHeight, 0, 0, 0, 0, 0,X_OFFSET},
        { 0, 0, StandHeight, 0, 0, 0, 0, 1,X_OFFSET},
        { 0, 0, StandHeight, 0, 0, 0, 0, 2,X_OFFSET},
        { 0, 0, StandHeight, 0, 0, 0, 0, 3,X_OFFSET},
    },
     //省略.....
    ```
2. ges_cal 中包含了最主要的代码逻辑
    -其中
    ```cpp
    void Gait(void)
    {
        double t = 10*HAL_GetTick()/1000.0f;

        switch (currentstate)
        {
        case Idle:  // 原地踏步
            
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
        // 转换到逆运动学的角度
        CartesianToTheta_Cycloid_All_Legs();
        // 控制腿部运动
        Moveleg();
    }
    ```
3.  Gait代码中:
    ```cpp
        double t = 10*HAL_GetTick()/1000.0f;
    ```
    - 通过HAL_GetTick()获取系统的实时时钟来作为机器狗运动的运动基础
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
    - 通过该正弦轨迹规划器规划机器狗的足端曲线，关于其中的一些变量已在ges_cal.c中有所描述
    - 通过识别currentstate状态---来自GaitParams.c中的 ```cpp State currentstate; ```实现切换步态状态的效果
    -最后通过 
    ```cpp  
        // 转换到逆运动学的角度
        CartesianToTheta_Cycloid_All_Legs();
        // 控制腿部运动
        Moveleg();
    ```
    来将足端轨迹通过逆运动学解算处电机所需角度，Moveleg()来将角度配置到各个腿的电机
4. 电机驱动代码
    - AllLeg_Set_angle()和Motor_Auto_Run()分别是用来设置电机角度的函数和用来执行电机运动的函数
    - 截取一小段代码 from Motor_Auto_Run()
    ```cpp
    //这是一段关于电机1的代码（总共八个电机）
     // ID1
    Motor_Angle_Cal_1(360); // 得到绝对角度
    Target_Pos_Setting(&pidmsg.M3508_POS_ID1, Leg_angle.motorangle1);
    motor_3508.ID1.angle_out = PID_Cal_POSITION(&pidmsg.M3508_POS_ID1, motor_3508.ID1.POS_ABS);
    motor_3508.ID1.target_speed = motor_3508.ID1.angle_out; // 角度环输出作为速度环输入
    motor_3508.ID1.current_output = PID_Calc(&pidmsg.M3508_SPEED_ID1, &motor_3508.ID1);
    canbuf[0] = ((short)(motor_3508.ID1.current_output)) >> 8;
    canbuf[1] = ((short)(motor_3508.ID1.current_output)) & 0x00FF;
    ```
5. jump相关代码
    - Start_Jump()用来获取开始跳跃的初始时间
    ```cpp
    void Start_Jump(void)
    {
        start_time = HAL_GetTick();
    }
    ```
    - Reset_start_time()用来在跳跃结束时重新获取下一个跳跃开始时的时间(来以此达到连续跳跃)
    ```cpp
    void Reset_start_time(void) 
    {
        start_time = HAL_GetTick();
    }
    ```
    - Execute_Jump()用来执行跳跃逻辑
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
                Jump_OK = 0;	//完成跳跃置零
                }
            }
        }

    }
    ```

## **控制执行逻辑**
- 在timer.c中包含了控制执行的逻辑
    - 其中在定时器4的中断函数中来执行Gait()代码模块
    ```cpp
    void TIM4_IRQHandler() //控制类逻辑
    {  
        if(TIM_GetITStatus(TIM4, TIM_IT_Update) !=RESET)
        {
            if(start==1)
            Gait(); //执行步态逻辑
            TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
        }
    ```
    - 而在定时器5中的中断函数用来执行控制模式的状态转换
    ```cpp
    void TIM5_IRQHandler(void) //状态机
    {
        if(TIM_GetITStatus(TIM5,TIM_IT_Update)!= RESET)
        {		

        switch(ctrl_state)
        {
                case Initial_Ctrl://初始状态 
                M3508_ALL_ZERO_SET();
                start=0;
                if_idle=0;
                break;
                case Start_Ctrl://初始化机器狗
                start=1;
                if_idle=0;
                Ctrl_Cmd();
                break;
                
                case Main_Ctrl://主控制
                if_idle=1;
                start=1;
                Ctrl_Cmd();
                break;
                
                case Stop_Ctrl:// 停止
                if_idle=0;
                start=1;
                Ctrl_Cmd();
                break;
                
                case Jump_Ctrl:// 跳跃
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
        TIM_ClearITPendingBit(TIM5,TIM_IT_Update); //清除中断标志位
    }
    ```
    - git控制模式的状态如下:
    ```cpp
    typedef enum 
    {
        Initial_Ctrl, //
        Start_Ctrl,
        Main_Ctrl,
        Stop_Ctrl,
        Jump_Ctrl,
    }Ctrl_State; //控制类状态机
    ```
## **远程控制代码---Remote_Ctrl文件**
1. RemoteControl_Init中包含了大疆遥控器接收数据的代码这里不做过多介绍(我还不怎么了解该部分)
2. RC_Command部分
    - Remote_Cmd()用来改变控制模块的状态
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
    - Ctrl_Cmd()用来通过遥控器传来的指令来改变currenstate即机器狗运动的状态(前进，后退，左右转等等)
    ```cpp
    void Ctrl_Cmd(void) //控制命令函数
    {
        //前进
        if (left_y >= 230 && abs(left_x) <= 330 && abs(right_y) <= 330 && abs(right_x) <= 330  ) 
        {	
            currentstate=Forward;
        }
        // 左平移
        else if (right_x <= -100 && abs(right_y) <= 230  && abs(left_y) <= 330 && abs(left_x) <= 330 )
        {
            
            currentstate=Translate_Left;
        }
        // 右平移
        else if (right_x >= 100  && abs(right_y) <= 230 && abs(left_y) <= 330 && abs(left_x) <= 330) 
        {
            
            currentstate=Translate_Right;
        }
        // 左转
        else if (left_x <= -100 && abs(left_y) <= 230  && abs(right_y) <= 330 && abs(right_x) <= 330 ) 
        {
            
            currentstate=Turn_Left;
        }
        // 右转
        else if (left_x >= 100  && abs(left_y) <= 230  && abs(right_y) <= 330 && abs(right_x) <= 330) 
        {
            
            currentstate=Turn_Right;
        }
        // 后退
        else if (left_y <= -230 && abs(left_x) <= 330 && abs(right_y) <= 330 && abs(right_x) <= 330 ) 
        {	
        
            currentstate=Back;
        }
        else if (right_y >= 230 && abs(right_x) <= 330  && abs(left_y) <= 330 && abs(left_x) <= 330 ) 
        {	
            currentstate=Jump;
        }

    // 重置 
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
## **IMU模块(IMU文件)**
1. 未完待续...