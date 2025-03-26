#ifndef __USART_H
#define __USART_H

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
void uart_init(uint32_t bound);
void uart3_init(uint32_t bound);
void USART1_IRQHandler(void);
void USART3_IRQHandler(void);
void UART1_Put_Char(unsigned char DataToSend);
void UART1_Put_String(unsigned char *Str);
void CopeSerial3Data(unsigned char ucData);

extern Euler_t Euler;
extern Accel_t Accel;
#endif // __USART_H
