#include "Jump.h"
double start_time;
int Jump_OK=1;
int Jump_Start=1;
void Start_Jump(void)
{
    start_time = HAL_GetTick();
}
void Execute_Jump(void)
{
    double t = 10.0*(HAL_GetTick()/1000.0f  - start_time/1000.0f);
   
    for (int i = 0; i < 4; i++) {
        if (t < 0.6) {

            double x = 3.1;
            double z = 11.59;
            legs[i].x = x;
            legs[i].z = z;
           //printf("waiting\n");
        }
        else if (t >= 0.6 && t < 1.0) {
            double x = 21.21;
            double z = 21.21;
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
//				Reset_start_time();
				Jump_OK = 0;	//Íê³ÉÌøÔ¾ÖÃÁã
		
            }
        }
    }

}
void Reset_start_time(void) {
    start_time = HAL_GetTick();
}