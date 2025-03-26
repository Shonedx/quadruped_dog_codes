#ifndef __IMU_H
#define __IMU_H
#include "Allheaderfile.h"
#include "sys.h"

#define IMU_REC_LEN 50


extern	double b1[3],a1[3],r1[3],r1_[3],	t1[3],t1_[3];
extern	double b2[3],a2[3],r2[3],r2_[3],	t2[3],t2_[3];
extern	double b3[3],a3[3],r3[3],r3_[3],	t3[3],t3_[3];
extern	double b4[3],a4[3],r4[3],r4_[3],	t4[3],t4_[3];
extern	double R[3][3];






void IMU_Control(void);
void Limit(double *t, double min ,double max);
void Limit_double(double t, double min ,double max);

#endif
