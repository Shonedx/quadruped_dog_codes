#ifndef __IMU_H
#define __IMU_H
#include <stdbool.h>
#include <stdint.h>
typedef struct 
{
	float yaw;
	float pitch;
	float roll;
}Adjust_Euler_Angle;
#define IMU_PACKET_SIZE 11
typedef struct 
{
	uint8_t buffer[IMU_PACKET_SIZE];
	uint8_t index;
	bool packet_ready;
}IMU_RxBuffer;

void IMU_Pos_Cal(double yaw, double pitch, double roll);
void setPostureOffset(void);
void calPosByRollNdPitch(float roll,float pitch);



#endif
