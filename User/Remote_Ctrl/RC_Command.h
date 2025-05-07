#ifndef __RC_COMMAND_H
#define __RC_COMMAND_H
#include "main.h"
#include "RemoteControl_Init.h"
//#define SBUS_RX_BUF_NUM 36u//36 �ֽ�

//#define RC_FRAME_LENGTH 18u// ��ܳ���


#define SBUS_RX_BUF_NUM   36u//36 �ֽ�

#define RC_FRAME_LENGTH   18u// ��ܳ���



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
//__packed :�ֽڶ���
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
**ARM7/9�ṹͨ��ϣ�����еĴ洢�����ʶ��������
*�ִ����ʼ��ַΪ4�ı���
*���ִ����ʼ��ַΪż��

*******************************************

**ARM Cortex-M��Ҫ�������ֺͰ��ֵĵ�ַ����
*ָ���ŵ�ַ����Ϊż��
*ĳЩ���������ȻҪ�����ݶ��룬���ջ��DMA��

��չ�ɹۿ� �й�Ľ�� ��Ƕ��ʽϵͳ��Ӧ�á�-������ͨ��ѧ

// ----------------------- �ڲ����� ----------------------------------- */

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
	Jump_Ctrl_1,
	Jump_Ctrl_2,
	Crouch_Ctrl,
	Higher_Ctrl,

}Ctrl_State; //������״̬��

extern Ctrl_State ctrl_state;
extern int if_idle; 
#endif
