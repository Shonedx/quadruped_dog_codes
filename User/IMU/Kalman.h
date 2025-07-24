// kalman.h
#ifndef KALMAN_H
#define KALMAN_H

// 对于欧拉角跟踪，使用2状态模型[角度, 角速度]
#define N_STATE 6
// 只测量角度
#define N_MEASUREMENT 3 //对应三个欧拉角



//移动平均滤波
typedef struct MovingAverageFilter {
    float *buffer;     // 数据缓冲区
    int size;          // 窗口大小
    int index;         // 当前写入位置
    float sum;         // 当前窗口内数据和
    int count;         // 当前数据计数(用于初始填充)
} MovingAverageFilter_t;
extern MovingAverageFilter_t yaw_filter;
extern MovingAverageFilter_t roll_filter;
extern MovingAverageFilter_t pitch_filter;

//移动平均滤波
void movAveInit(MovingAverageFilter_t *filter, int window_size);
float movAveUpdate(MovingAverageFilter_t *filter, float new_value);
void movAveFree(MovingAverageFilter_t *filter);
#endif // KALMAN_H