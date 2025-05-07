#include "kalman.h"
#include <math.h>   // ���� fabs()
#include <string.h> // ���� memcpy

// --- ��̬���������� ---
// ��Щ����ʹ�ñ�ƽ�� float �������ʽ��ά�����ṩͨ���ԡ�

// ����˷�������C = A * B
// A �� rowsA x colsA��B �� rowsB x colsB��C �� rowsA x colsB��
// colsA ������� rowsB��
static void mat_mult(const float* A, int rowsA, int colsA,
                       const float* B, int rowsB, int colsB,
                       float* C) {
    if (colsA != rowsB) {
        // ά�Ȳ�ƥ�䣬�����������
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

// ����ת�ú�����AT = A^T
// A �� rows x cols��AT �� cols x rows��
static void mat_transpose(const float* A, int rows, int cols, float* AT) {
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            AT[j * rows + i] = A[i * cols + j];
        }
    }
}

// �������������C = A - B
// A, B, C ���� rows x cols��
static void mat_subtract(const float* A, const float* B, int rows, int cols, float* C) {
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            C[i * cols + j] = A[i * cols + j] - B[i * cols + j];
        }
    }
}

// ����ӷ�������C = A + B
// A, B, C ���� rows x cols��
static void mat_add(const float* A, const float* B, int rows, int cols, float* C) {
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            C[i * cols + j] = A[i * cols + j] + B[i * cols + j];
        }
    }
}

// ������λ������
static void mat_identity(int rows, int cols, float* I) {
    if (rows != cols) {
         // ���Ƿ��󣬲����ǳ��������ϵĵ�λ����
         return;
    }
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            I[i * cols + j] = (i == j) ? 1.0f : 0.0f;
        }
    }
}


// --- �������˲�������ʵ�� ---

void KalmanFilter_Init(KalmanFilter* kf,
                       float initial_angle,
                       float initial_angular_velocity,
                       float dt,
                       float process_noise_variance,
                       float measurement_noise_variance,
                       float initial_covariance) {

    // �洢����
    kf->dt = dt;
    kf->process_noise_variance = process_noise_variance;
    kf->measurement_noise_variance = measurement_noise_variance;

    // ��ʼ��״̬���� [�Ƕ�, ���ٶ�]
    kf->x[0] = initial_angle;
    kf->x[1] = initial_angular_velocity;
    // ��� N_STATE > 2��ȷ������״̬Ԫ��Ϊ��
    for (int i = 2; i < N_STATE; ++i) {
        kf->x[i] = 0.0f;
    }


    // ��ʼ��״̬Э������� P (��λ������� initial_covariance)
    for (int i = 0; i < N_STATE; i++) {
        for (int j = 0; j < N_STATE; j++) {
            kf->P[i][j] = (i == j) ? initial_covariance : 0.0f;
        }
    }

    // ��ʼ��״̬ת�ƾ��� F
    // ��������ģ�ͣ�
    // [ 1  dt ]
    // [ 0  1  ]
    // ��� N_STATE > 2������Ҫ����
    mat_identity(N_STATE, N_STATE, &kf->F[0][0]); // �ӵ�λ����ʼ
    if (N_STATE >= 2) {
        kf->F[0][1] = dt; // ���״̬����λ��/�ٶȣ��������Ͻ���� dt
    }
    // ��� N_STATE > 2���������ģ���������ת����

    // ��ʼ����������Э������� Q
    // �򵥳�ʼ�����ԽǾ��󣬶Խ���ֵΪ process_noise_variance
    // ���ڸ����ӵ�ģ�ͣ�Q ���ܲ�ͬ
    for (int i = 0; i < N_STATE; i++) {
        for (int j = 0; j < N_STATE; j++) {
            kf->Q[i][j] = (i == j) ? process_noise_variance : 0.0f;
        }
    }
    // ���� N_STATE=2��[λ��, �ٶ�] �ĸ������������ Q ������ [dt^3/3 dt^2/2; dt^2/2 dt] * ���ٶ���������
    // �������Ҫ������滻�����ѭ���������һ��������
    // ���磬���� N_STATE=2��[λ��, �ٶ�]��
    // kf->Q[0][0] = process_noise_variance * (dt*dt*dt/3.0f);
    // kf->Q[0][1] = process_noise_variance * (dt*dt/2.0f);
    // kf->Q[1][0] = process_noise_variance * (dt*dt/2.0f);
    // kf->Q[1][1] = process_noise_variance * dt;


    // ��ʼ���۲���� H
    // ���ǲ����Ƕ� (��һ��״̬����)
    // [ 1  0 ... ]
    // ���� measurement[0] �ǽǶ�
    for (int i = 0; i < N_MEASUREMENT; i++) {
         for (int j = 0; j < N_STATE; j++) {
             kf->H[i][j] = 0.0f;
         }
    }
    if (N_MEASUREMENT >= 1 && N_STATE >= 1) {
         kf->H[0][0] = 1.0f; // ���ǲ�����һ��״̬���� (�Ƕ�)
    }
    // ��� N_MEASUREMENT > 1������������ı����������� H ��Ŀ

    // ��ʼ����������Э������� R
    // �򵥳�ʼ�����ԽǾ��󣬶Խ���ֵΪ measurement_noise_variance
    // ��� N_MEASUREMENT > 1��R ���� N_MEASUREMENT x N_MEASUREMENT
    for (int i = 0; i < N_MEASUREMENT; i++) {
         for (int j = 0; j < N_MEASUREMENT; j++) {
             kf->R[i][j] = (i == j) ? measurement_noise_variance : 0.0f;
         }
    }

    // ���������� K �ڸ��²������Ժ��ʼ��
}

void KalmanFilter_Predict(KalmanFilter* kf) {
    // --- Ԥ�ⲽ�� ---
    // 1. Ԥ����һ��״̬��x_k|k-1 = F * x_k-1|k-1
    //    ���� x ��һ�������� (N_STATE x 1)��ʹ�� mat_mult
    float predicted_x[N_STATE];
    mat_mult(&kf->F[0][0], N_STATE, N_STATE,
             &kf->x[0], N_STATE, 1,
             predicted_x);
    // ��������ƻ� kf->x
    memcpy(kf->x, predicted_x, sizeof(predicted_x));


    // 2. Ԥ����һ��Э���P_k|k-1 = F * P_k-1|k-1 * F^T + Q
    float FT[N_STATE][N_STATE];
    mat_transpose(&kf->F[0][0], N_STATE, N_STATE, &FT[0][0]);

    float FP[N_STATE][N_STATE];
    mat_mult(&kf->F[0][0], N_STATE, N_STATE,
             &kf->P[0][0], N_STATE, N_STATE,
             &FP[0][0]);

    float FPF_T[N_STATE][N_STATE];
     mat_mult(&FP[0][0], N_STATE, N_STATE,
             &FT[0][0], N_STATE, N_STATE,
             &FPF_T[0][0]); // ʹ�� FPF_T ��Ϊ��ʱ�洢

    // ��ӹ�������Э���P = F*P*F' + Q
    mat_add(&FPF_T[0][0], &kf->Q[0][0], N_STATE, N_STATE, &kf->P[0][0]);

    // kf �е�״̬������Э�������������Ԥ��ֵ (k|k-1)
}

void KalmanFilter_Update(KalmanFilter* kf, const float measurement[N_MEASUREMENT]) {
    // --- ���²��� ---
    // ʹ������Ԥ�ⲽ���Ԥ��״̬ (kf->x) ��Э���� (kf->P)

    // 1. ��������в� (��Ϣ)��y_k = z_k - H * x_k|k-1
    float Hx[N_MEASUREMENT];
    mat_mult(&kf->H[0][0], N_MEASUREMENT, N_STATE,
             &kf->x[0], N_STATE, 1,
             Hx); // H*x ��һ�� N_MEASUREMENT x 1 ������

    float residual[N_MEASUREMENT]; // y_k ��һ������
    // residual = measurement - Hx
    for(int i = 0; i < N_MEASUREMENT; i++) {
        residual[i] = measurement[i] - Hx[i];
    }


    // 2. ����в�Э���� (��ϢЭ����)��S_k = H * P_k|k-1 * H^T + R
    float HT[N_STATE][N_MEASUREMENT];
    mat_transpose(&kf->H[0][0], N_MEASUREMENT, N_STATE, &HT[0][0]);

    float PHt[N_STATE][N_MEASUREMENT];
    mat_mult(&kf->P[0][0], N_STATE, N_STATE,
             &HT[0][0], N_STATE, N_MEASUREMENT,
             &PHt[0][0]); // P * H^T �� N_STATE x N_MEASUREMENT

    float HPHt[N_MEASUREMENT][N_MEASUREMENT];
    mat_mult(&kf->H[0][0], N_MEASUREMENT, N_STATE,
             &PHt[0][0], N_STATE, N_MEASUREMENT,
             &HPHt[0][0]); // H * P * H^T �� N_MEASUREMENT x N_MEASUREMENT

    float S_matrix[N_MEASUREMENT][N_MEASUREMENT];
    mat_add(&HPHt[0][0], &kf->R[0][0], N_MEASUREMENT, N_MEASUREMENT, &S_matrix[0][0]);

    // ���� N_MEASUREMENT = 1��S ��һ����������Ҫ S ���档
    // ���� N_MEASUREMENT > 1��������Ҫһ��ͨ�õľ������溯����
    // ʵ��ͨ�õľ������� (���磬ʹ�ø�˹��Ԫ��) ���������Ӹ����Ժͼ���ɱ���
    // ��������ض����� (ŷ���ǣ��ܿ��� N_MEASUREMENT=1)��
    // S �� 1x1������ S_matrix[0][0] �Ǳ���ֵ��S ������ 1/S��

    float S_inverse[N_MEASUREMENT][N_MEASUREMENT]; // S �������
    // N_MEASUREMENT == 1 ������
    if (N_MEASUREMENT == 1) {
        float S_scalar = S_matrix[0][0];
        if (fabs(S_scalar) < 1e-9) { // ����Ƿ����� (S Ϊ���ӽ���)
             // ����ֵ��Ԥ�� + ����ģ�Ͳ�һ�¡�
             // �����������£��������˲�����Ŀǰ��ֱ�ӷ��ء�
             return;
        }
        S_inverse[0][0] = 1.0f / S_scalar;
    } else {
        // TODO: Ϊ N_MEASUREMENT > 1 ʵ�־�������
        // ����һ���ش�Ĳ��䡣���ڱ�ʾ�������Ǽ��� N_MEASUREMENT = 1��
        return; // û�о������棬�޷�ִ�и���
    }


    // 3. ���㿨�������棺K_k = P_k|k-1 * H^T * S_k^-1
    // PHt �Ѿ�������� (P * H^T��N_STATE x N_MEASUREMENT)
    // K = PHt * S_inverse
    mat_mult(&PHt[0][0], N_STATE, N_MEASUREMENT,
             &S_inverse[0][0], N_MEASUREMENT, N_MEASUREMENT,
             &kf->K[0][0]); // K �� N_STATE x N_MEASUREMENT


    // 4. ����״̬���ƣ�x_k|k = x_k|k-1 + K_k * y_k
    float Ky[N_STATE];
    // K �� N_STATE x N_MEASUREMENT���в� (y) �� N_MEASUREMENT x 1
    mat_mult(&kf->K[0][0], N_STATE, N_MEASUREMENT,
             residual, N_MEASUREMENT, 1,
             Ky); // K*y �� N_STATE x 1

    // x = x + Ky
    for(int i = 0; i < N_STATE; i++) {
        kf->x[i] += Ky[i];
    }


    // 5. ����Э������ƣ�P_k|k = (I - K_k * H) * P_k|k-1
    float KH[N_STATE][N_STATE];
    mat_mult(&kf->K[0][0], N_STATE, N_MEASUREMENT,
             &kf->H[0][0], N_MEASUREMENT, N_STATE,
             &KH[0][0]); // K*H �� N_STATE x N_STATE

    float I_matrix[N_STATE][N_STATE];
    mat_identity(N_STATE, N_STATE, &I_matrix[0][0]);

    float I_KH[N_STATE][N_STATE];
    mat_subtract(&I_matrix[0][0], &KH[0][0], N_STATE, N_STATE, &I_KH[0][0]); // I - K*H

    // P = (I - K*H) * P
    float updated_P[N_STATE][N_STATE]; // ʹ����ʱ�洢��Ž��
    mat_mult(&I_KH[0][0], N_STATE, N_STATE,
             &kf->P[0][0], N_STATE, N_STATE,
             &updated_P[0][0]);

    // �����º��Э����ƻ� kf->P
    memcpy(kf->P, updated_P, sizeof(kf->P));

    // kf �е�״̬���� (kf->x) ��Э������� (kf->P) �����Ǹ���ֵ (k|k)
}