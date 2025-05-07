#include "kalman.h"
#include <math.h>   // 用于 fabs()
#include <string.h> // 用于 memcpy

// --- 静态矩阵辅助函数 ---
// 这些函数使用扁平的 float 数组和显式的维度来提供通用性。

// 矩阵乘法函数：C = A * B
// A 是 rowsA x colsA，B 是 rowsB x colsB。C 是 rowsA x colsB。
// colsA 必须等于 rowsB。
static void mat_mult(const float* A, int rowsA, int colsA,
                       const float* B, int rowsB, int colsB,
                       float* C) {
    if (colsA != rowsB) {
        // 维度不匹配，处理错误或断言
        return;
    }
    for (int i = 0; i < rowsA; i++) {
        for (int j = 0; j < colsB; j++) {
            C[i * colsB + j] = 0;
            for (int k = 0; k < colsA; k++) {
                C[i * colsB + j] += A[i * colsA + k] * B[k * colsB + j];
            }
        }
    }
}

// 矩阵转置函数：AT = A^T
// A 是 rows x cols。AT 是 cols x rows。
static void mat_transpose(const float* A, int rows, int cols, float* AT) {
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            AT[j * rows + i] = A[i * cols + j];
        }
    }
}

// 矩阵减法函数：C = A - B
// A, B, C 都是 rows x cols。
static void mat_subtract(const float* A, const float* B, int rows, int cols, float* C) {
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            C[i * cols + j] = A[i * cols + j] - B[i * cols + j];
        }
    }
}

// 矩阵加法函数：C = A + B
// A, B, C 都是 rows x cols。
static void mat_add(const float* A, const float* B, int rows, int cols, float* C) {
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            C[i * cols + j] = A[i * cols + j] + B[i * cols + j];
        }
    }
}

// 创建单位矩阵函数
static void mat_identity(int rows, int cols, float* I) {
    if (rows != cols) {
         // 不是方阵，不能是常规意义上的单位矩阵
         return;
    }
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            I[i * cols + j] = (i == j) ? 1.0f : 0.0f;
        }
    }
}


// --- 卡尔曼滤波器函数实现 ---

void KalmanFilter_Init(KalmanFilter* kf,
                       float initial_angle,
                       float initial_angular_velocity,
                       float dt,
                       float process_noise_variance,
                       float measurement_noise_variance,
                       float initial_covariance) {

    // 存储参数
    kf->dt = dt;
    kf->process_noise_variance = process_noise_variance;
    kf->measurement_noise_variance = measurement_noise_variance;

    // 初始化状态向量 [角度, 角速度]
    kf->x[0] = initial_angle;
    kf->x[1] = initial_angular_velocity;
    // 如果 N_STATE > 2，确保其余状态元素为零
    for (int i = 2; i < N_STATE; ++i) {
        kf->x[i] = 0.0f;
    }


    // 初始化状态协方差矩阵 P (单位矩阵乘以 initial_covariance)
    for (int i = 0; i < N_STATE; i++) {
        for (int j = 0; j < N_STATE; j++) {
            kf->P[i][j] = (i == j) ? initial_covariance : 0.0f;
        }
    }

    // 初始化状态转移矩阵 F
    // 对于匀速模型：
    // [ 1  dt ]
    // [ 0  1  ]
    // 如果 N_STATE > 2，这需要调整
    mat_identity(N_STATE, N_STATE, &kf->F[0][0]); // 从单位矩阵开始
    if (N_STATE >= 2) {
        kf->F[0][1] = dt; // 如果状态包含位置/速度，则在右上角添加 dt
    }
    // 如果 N_STATE > 2，根据你的模型添加其他转移项

    // 初始化过程噪声协方差矩阵 Q
    // 简单初始化：对角矩阵，对角线值为 process_noise_variance
    // 对于更复杂的模型，Q 可能不同
    for (int i = 0; i < N_STATE; i++) {
        for (int j = 0; j < N_STATE; j++) {
            kf->Q[i][j] = (i == j) ? process_noise_variance : 0.0f;
        }
    }
    // 对于 N_STATE=2，[位置, 速度] 的更具物理意义的 Q 可以是 [dt^3/3 dt^2/2; dt^2/2 dt] * 加速度噪声方差
    // 如果你需要这个，替换上面的循环或添加另一个函数。
    // 例如，对于 N_STATE=2，[位置, 速度]：
    // kf->Q[0][0] = process_noise_variance * (dt*dt*dt/3.0f);
    // kf->Q[0][1] = process_noise_variance * (dt*dt/2.0f);
    // kf->Q[1][0] = process_noise_variance * (dt*dt/2.0f);
    // kf->Q[1][1] = process_noise_variance * dt;


    // 初始化观测矩阵 H
    // 我们测量角度 (第一个状态变量)
    // [ 1  0 ... ]
    // 假设 measurement[0] 是角度
    for (int i = 0; i < N_MEASUREMENT; i++) {
         for (int j = 0; j < N_STATE; j++) {
             kf->H[i][j] = 0.0f;
         }
    }
    if (N_MEASUREMENT >= 1 && N_STATE >= 1) {
         kf->H[0][0] = 1.0f; // 我们测量第一个状态变量 (角度)
    }
    // 如果 N_MEASUREMENT > 1，根据你测量的变量设置其他 H 条目

    // 初始化测量噪声协方差矩阵 R
    // 简单初始化：对角矩阵，对角线值为 measurement_noise_variance
    // 如果 N_MEASUREMENT > 1，R 将是 N_MEASUREMENT x N_MEASUREMENT
    for (int i = 0; i < N_MEASUREMENT; i++) {
         for (int j = 0; j < N_MEASUREMENT; j++) {
             kf->R[i][j] = (i == j) ? measurement_noise_variance : 0.0f;
         }
    }

    // 卡尔曼增益 K 在更新步骤中稍后初始化
}

void KalmanFilter_Predict(KalmanFilter* kf) {
    // --- 预测步骤 ---
    // 1. 预测下一个状态：x_k|k-1 = F * x_k-1|k-1
    //    由于 x 是一个列向量 (N_STATE x 1)，使用 mat_mult
    float predicted_x[N_STATE];
    mat_mult(&kf->F[0][0], N_STATE, N_STATE,
             &kf->x[0], N_STATE, 1,
             predicted_x);
    // 将结果复制回 kf->x
    memcpy(kf->x, predicted_x, sizeof(predicted_x));


    // 2. 预测下一个协方差：P_k|k-1 = F * P_k-1|k-1 * F^T + Q
    float FT[N_STATE][N_STATE];
    mat_transpose(&kf->F[0][0], N_STATE, N_STATE, &FT[0][0]);

    float FP[N_STATE][N_STATE];
    mat_mult(&kf->F[0][0], N_STATE, N_STATE,
             &kf->P[0][0], N_STATE, N_STATE,
             &FP[0][0]);

    float FPF_T[N_STATE][N_STATE];
     mat_mult(&FP[0][0], N_STATE, N_STATE,
             &FT[0][0], N_STATE, N_STATE,
             &FPF_T[0][0]); // 使用 FPF_T 作为临时存储

    // 添加过程噪声协方差：P = F*P*F' + Q
    mat_add(&FPF_T[0][0], &kf->Q[0][0], N_STATE, N_STATE, &kf->P[0][0]);

    // kf 中的状态向量和协方差矩阵现在是预测值 (k|k-1)
}

void KalmanFilter_Update(KalmanFilter* kf, const float measurement[N_MEASUREMENT]) {
    // --- 更新步骤 ---
    // 使用来自预测步骤的预测状态 (kf->x) 和协方差 (kf->P)

    // 1. 计算测量残差 (新息)：y_k = z_k - H * x_k|k-1
    float Hx[N_MEASUREMENT];
    mat_mult(&kf->H[0][0], N_MEASUREMENT, N_STATE,
             &kf->x[0], N_STATE, 1,
             Hx); // H*x 是一个 N_MEASUREMENT x 1 的向量

    float residual[N_MEASUREMENT]; // y_k 是一个向量
    // residual = measurement - Hx
    for(int i = 0; i < N_MEASUREMENT; i++) {
        residual[i] = measurement[i] - Hx[i];
    }


    // 2. 计算残差协方差 (新息协方差)：S_k = H * P_k|k-1 * H^T + R
    float HT[N_STATE][N_MEASUREMENT];
    mat_transpose(&kf->H[0][0], N_MEASUREMENT, N_STATE, &HT[0][0]);

    float PHt[N_STATE][N_MEASUREMENT];
    mat_mult(&kf->P[0][0], N_STATE, N_STATE,
             &HT[0][0], N_STATE, N_MEASUREMENT,
             &PHt[0][0]); // P * H^T 是 N_STATE x N_MEASUREMENT

    float HPHt[N_MEASUREMENT][N_MEASUREMENT];
    mat_mult(&kf->H[0][0], N_MEASUREMENT, N_STATE,
             &PHt[0][0], N_STATE, N_MEASUREMENT,
             &HPHt[0][0]); // H * P * H^T 是 N_MEASUREMENT x N_MEASUREMENT

    float S_matrix[N_MEASUREMENT][N_MEASUREMENT];
    mat_add(&HPHt[0][0], &kf->R[0][0], N_MEASUREMENT, N_MEASUREMENT, &S_matrix[0][0]);

    // 对于 N_MEASUREMENT = 1，S 是一个标量。需要 S 的逆。
    // 对于 N_MEASUREMENT > 1，这里需要一个通用的矩阵求逆函数。
    // 实现通用的矩阵求逆 (例如，使用高斯消元法) 会显著增加复杂性和计算成本。
    // 对于这个特定问题 (欧拉角，很可能 N_MEASUREMENT=1)，
    // S 是 1x1，所以 S_matrix[0][0] 是标量值。S 的逆是 1/S。

    float S_inverse[N_MEASUREMENT][N_MEASUREMENT]; // S 的逆矩阵
    // N_MEASUREMENT == 1 的特例
    if (N_MEASUREMENT == 1) {
        float S_scalar = S_matrix[0][0];
        if (fabs(S_scalar) < 1e-9) { // 检查是否奇异 (S 为零或接近零)
             // 测量值与预测 + 噪声模型不一致。
             // 可以跳过更新，或重置滤波器。目前，直接返回。
             return;
        }
        S_inverse[0][0] = 1.0f / S_scalar;
    } else {
        // TODO: 为 N_MEASUREMENT > 1 实现矩阵求逆
        // 这是一个重大的补充。对于本示例，我们假设 N_MEASUREMENT = 1。
        return; // 没有矩阵求逆，无法执行更新
    }


    // 3. 计算卡尔曼增益：K_k = P_k|k-1 * H^T * S_k^-1
    // PHt 已经计算过了 (P * H^T，N_STATE x N_MEASUREMENT)
    // K = PHt * S_inverse
    mat_mult(&PHt[0][0], N_STATE, N_MEASUREMENT,
             &S_inverse[0][0], N_MEASUREMENT, N_MEASUREMENT,
             &kf->K[0][0]); // K 是 N_STATE x N_MEASUREMENT


    // 4. 更新状态估计：x_k|k = x_k|k-1 + K_k * y_k
    float Ky[N_STATE];
    // K 是 N_STATE x N_MEASUREMENT，残差 (y) 是 N_MEASUREMENT x 1
    mat_mult(&kf->K[0][0], N_STATE, N_MEASUREMENT,
             residual, N_MEASUREMENT, 1,
             Ky); // K*y 是 N_STATE x 1

    // x = x + Ky
    for(int i = 0; i < N_STATE; i++) {
        kf->x[i] += Ky[i];
    }


    // 5. 更新协方差估计：P_k|k = (I - K_k * H) * P_k|k-1
    float KH[N_STATE][N_STATE];
    mat_mult(&kf->K[0][0], N_STATE, N_MEASUREMENT,
             &kf->H[0][0], N_MEASUREMENT, N_STATE,
             &KH[0][0]); // K*H 是 N_STATE x N_STATE

    float I_matrix[N_STATE][N_STATE];
    mat_identity(N_STATE, N_STATE, &I_matrix[0][0]);

    float I_KH[N_STATE][N_STATE];
    mat_subtract(&I_matrix[0][0], &KH[0][0], N_STATE, N_STATE, &I_KH[0][0]); // I - K*H

    // P = (I - K*H) * P
    float updated_P[N_STATE][N_STATE]; // 使用临时存储存放结果
    mat_mult(&I_KH[0][0], N_STATE, N_STATE,
             &kf->P[0][0], N_STATE, N_STATE,
             &updated_P[0][0]);

    // 将更新后的协方差复制回 kf->P
    memcpy(kf->P, updated_P, sizeof(kf->P));

    // kf 中的状态向量 (kf->x) 和协方差矩阵 (kf->P) 现在是更新值 (k|k)
}