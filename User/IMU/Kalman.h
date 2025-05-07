#ifndef KALMAN_H
#define KALMAN_H

#include <stddef.h> // 用于 size_t

// --- 配置宏 ---
// 定义状态向量的大小 (例如，2 表示 [角度, 角速度])
#define N_STATE 2
// 定义测量向量的大小 (例如，1 表示 [角度测量值])
#define N_MEASUREMENT 1
// ----------------------------

// 卡尔曼滤波器结构体，包含状态和参数
typedef struct {
    float x[N_STATE];       // 状态向量 [角度, 角速度]
    float P[N_STATE][N_STATE];   // 状态协方差矩阵
    float F[N_STATE][N_STATE];   // 状态转移矩阵
    float Q[N_STATE][N_STATE];   // 过程噪声协方差矩阵
    float H[N_MEASUREMENT][N_STATE]; // 观测矩阵
    float R[N_MEASUREMENT][N_MEASUREMENT]; // 观测噪声协方差矩阵
    float K[N_STATE][N_MEASUREMENT]; // 卡尔曼增益矩阵 (内部使用，存储以便检查)

    // 参数 (可选，可能只在 Init 中使用)
    float dt; // 时间步长
    float process_noise_variance; // 如果 Q 是对角矩阵，这是对角线上的方差标量
    float measurement_noise_variance; // 如果 R 是对角矩阵，这是对角线上的方差标量

} KalmanFilter;

// --- 卡尔曼滤波器函数 ---

/**
 * @brief 初始化卡尔曼滤波器。
 *
 * @param kf 指向 KalmanFilter 结构体的指针。
 * @param initial_angle 初始角度估计值。
 * @param initial_angular_velocity 初始角速度估计值。
 * @param dt 预测之间的时间步长。
 * @param process_noise_variance 过程噪声对角线上的方差 (假设 Q 是对角矩阵)。
 * @param measurement_noise_variance 测量噪声对角线上的方差 (假设 R 是对角矩阵)。
 * @param initial_covariance 初始对角线上的协方差 (例如，1.0)。
 */
void KalmanFilter_Init(KalmanFilter* kf,
                       float initial_angle,
                       float initial_angular_velocity,
                       float dt,
                       float process_noise_variance,
                       float measurement_noise_variance,
                       float initial_covariance);

/**
 * @brief 执行卡尔曼滤波器的预测步骤。
 *
 * @param kf 指向 KalmanFilter 结构体的指针。
 */
void KalmanFilter_Predict(KalmanFilter* kf);

/**
 * @brief 使用新的测量值执行卡尔曼滤波器的更新步骤。
 *
 * @param kf 指向 KalmanFilter 结构体的指针。
 * @param measurement 测量向量 (大小为 N_MEASUREMENT 的 float 数组)。
 */
void KalmanFilter_Update(KalmanFilter* kf, const float measurement[N_MEASUREMENT]);

#endif // KALMAN_H