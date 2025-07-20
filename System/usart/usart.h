#ifndef __USART_H
#define __USART_H
#include "stdint.h"
// USART��غ궨��
#define EN_USART1_RX 1 // �Ƿ�����USART1��RX�жϡ�
// ���ݽṹ����
typedef struct {
    float roll;
    float pitch;
    float yaw;
} Euler_t;
typedef struct {
    float x;
    float y;
    float z;
} Accel_t;
typedef struct {
    int16_t Angle[3];
} stcAngle_t;
typedef struct {
    int16_t Accel[3];
} stcAccel_t;
// USART��غ�������
void USART1_Init(void);
void USART4_Init(void);
void USART3_Init(void);

void UART1_Put_Char(unsigned char DataToSend);
void UART1_Put_String(unsigned char *Str);
void CopeSerial3Data(unsigned char ucData);
void usart1TxDateToVofa(float ch1, float ch2, float ch3,float ch4);



extern Euler_t Euler;
extern Accel_t Accel;
#endif // __USART_H
