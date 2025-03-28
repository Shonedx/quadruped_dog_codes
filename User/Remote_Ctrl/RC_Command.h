#ifndef __RC_COMMAND_H
#define __RC_COMMAND_H
#include "main.h"
#include "RemoteControl_Init.h"
//#define SBUS_RX_BUF_NUM 36u//36 字节

//#define RC_FRAME_LENGTH 18u// 框架长度


#define SBUS_RX_BUF_NUM   36u//36 字节

#define RC_FRAME_LENGTH   18u// 框架长度



/* ----------------------- RC Channel Definition----------------------------- */
#define RC_CH_VALUE_MIN ((uint16_t)364)
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define RC_CH_VALUE_MAX ((uint16_t)1684)


/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP ((uint16_t)1)
#define RC_SW_MID ((uint16_t)3)
#define RC_SW_DOWN ((uint16_t)2)

#define switch_is_down(s) (s == RC_SW_DOWN)
#define switch_is_mid(s) (s == RC_SW_MID)
#define switch_is_up(s) (s == RC_SW_UP)

/* ----------------------- PC Key Definition-------------------------------- */

#define KEY_PRESSED_OFFSET_W ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B ((uint16_t)1 << 15)

/* ----------------------- Data Struct ------------------------------------- */
//__packed :字节对齐
typedef __packed struct
{
        __packed struct
        {
                int16_t ch[5];
                char s[2];
        } rc;
        __packed struct
        {
                int16_t x;
                int16_t y;
                int16_t z;
                uint8_t press_l;
                uint8_t press_r;
        } mouse;
        __packed struct
        {
                uint16_t v;
        } key;

} RC_ctrl_t;

/******************************************
**ARM7/9结构通常希望所有的存储器访问都合理对齐
*字存放起始地址为4的倍数
*半字存放起始地址为偶数

*******************************************

**ARM Cortex-M不要求数据字和半字的地址对齐
*指令存放地址必须为偶数
*某些特殊操作仍然要求数据对齐，如堆栈，DMA等

扩展可观看 中国慕课 《嵌入式系统及应用》-北京交通大学

// ----------------------- 内部数据 ----------------------------------- */

extern void remote_control_init(void);
extern const RC_ctrl_t *get_remote_control_point(void);
extern uint8_t RC_data_is_error(void);
extern void slove_RC_lost(void);
extern void slove_data_error(void);
static RC_ctrl_t rc_ctrl;



void Remote_Cmd(void);
void Ctrl_Cmd(void);

typedef enum 
{
	Initial_Ctrl,
	Start_Ctrl,
	Main_Ctrl,
	Stop_Ctrl,
	Jump_Ctrl,
}Ctrl_State; //控制类状态机

extern Ctrl_State ctrl_state;
extern int if_idle; 
#endif
