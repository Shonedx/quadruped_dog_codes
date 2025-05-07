#ifndef KALMAN_H
#define KALMAN_H

#include <stddef.h> // ���� size_t

// --- ���ú� ---
// ����״̬�����Ĵ�С (���磬2 ��ʾ [�Ƕ�, ���ٶ�])
#define N_STATE 2
// ������������Ĵ�С (���磬1 ��ʾ [�ǶȲ���ֵ])
#define N_MEASUREMENT 1
// ----------------------------

// �������˲����ṹ�壬����״̬�Ͳ���
typedef struct {
    float x[N_STATE];       // ״̬���� [�Ƕ�, ���ٶ�]
    float P[N_STATE][N_STATE];   // ״̬Э�������
    float F[N_STATE][N_STATE];   // ״̬ת�ƾ���
    float Q[N_STATE][N_STATE];   // ��������Э�������
    float H[N_MEASUREMENT][N_STATE]; // �۲����
    float R[N_MEASUREMENT][N_MEASUREMENT]; // �۲�����Э�������
    float K[N_STATE][N_MEASUREMENT]; // ������������� (�ڲ�ʹ�ã��洢�Ա���)

    // ���� (��ѡ������ֻ�� Init ��ʹ��)
    float dt; // ʱ�䲽��
    float process_noise_variance; // ��� Q �ǶԽǾ������ǶԽ����ϵķ������
    float measurement_noise_variance; // ��� R �ǶԽǾ������ǶԽ����ϵķ������

} KalmanFilter;

// --- �������˲������� ---

/**
 * @brief ��ʼ���������˲�����
 *
 * @param kf ָ�� KalmanFilter �ṹ���ָ�롣
 * @param initial_angle ��ʼ�Ƕȹ���ֵ��
 * @param initial_angular_velocity ��ʼ���ٶȹ���ֵ��
 * @param dt Ԥ��֮���ʱ�䲽����
 * @param process_noise_variance ���������Խ����ϵķ��� (���� Q �ǶԽǾ���)��
 * @param measurement_noise_variance ���������Խ����ϵķ��� (���� R �ǶԽǾ���)��
 * @param initial_covariance ��ʼ�Խ����ϵ�Э���� (���磬1.0)��
 */
void KalmanFilter_Init(KalmanFilter* kf,
                       float initial_angle,
                       float initial_angular_velocity,
                       float dt,
                       float process_noise_variance,
                       float measurement_noise_variance,
                       float initial_covariance);

/**
 * @brief ִ�п������˲�����Ԥ�ⲽ�衣
 *
 * @param kf ָ�� KalmanFilter �ṹ���ָ�롣
 */
void KalmanFilter_Predict(KalmanFilter* kf);

/**
 * @brief ʹ���µĲ���ִֵ�п������˲����ĸ��²��衣
 *
 * @param kf ָ�� KalmanFilter �ṹ���ָ�롣
 * @param measurement �������� (��СΪ N_MEASUREMENT �� float ����)��
 */
void KalmanFilter_Update(KalmanFilter* kf, const float measurement[N_MEASUREMENT]);

#endif // KALMAN_H