// kalman.c
#include "kalman.h"
#include <math.h>
#include <string.h>
#include "../DEFINE/define_file.h"
MovingAverageFilter_t yaw_filter;
MovingAverageFilter_t roll_filter;
MovingAverageFilter_t pitch_filter;
//移动平均滤波
void movAveInit(MovingAverageFilter_t *filter, int window_size) {
    filter->size = window_size;
    filter->buffer = (float *)malloc(window_size * sizeof(float));
    filter->index = 0;
    filter->sum = 0;
    filter->count = 0;
    memset(filter->buffer, 0, window_size * sizeof(float));
}

float movAveUpdate(MovingAverageFilter_t *filter, float new_value) {
    // 如果缓冲区已满，减去即将被替换的值
    if (filter->count >= filter->size) {
        filter->sum -= filter->buffer[filter->index];
    } else {
        filter->count++;
    }
    
    // 添加新值
    filter->buffer[filter->index] = new_value;
    filter->sum += new_value;
    
    // 更新索引
    filter->index = (filter->index + 1) % filter->size;
    
    // 返回平均值
    return filter->sum / filter->count;
}

void movAveFree(MovingAverageFilter_t *filter) {
    free(filter->buffer);
}
