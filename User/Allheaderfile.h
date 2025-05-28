#include	"sys.h"
#include <stdio.h>
#include <stdlib.h> 
#include "math.h"
#include	"string.h"

#include	"delay.h"
#include	"usart.h"

#include	"led.h"
#include	"can.h"
#include	"pid.h"
#include 	"motor.h"
#include	"IMU.h"
#include	"timer.h"
#include    "OLED.h"
#include 	"nrf24l01.h"

#include	"RemoteControl_Init.h"
#include	"RC_Command.h"

#include	"IWDG.h"

#include	"ges_cal.h"
#include "GaitParams.h"
#include "Jump.h"

#include	"main_params.h"

#define LPS left_push_stick 	//左上推杆
#define RPS right_push_stick 	//右上推杆

