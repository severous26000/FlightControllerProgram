#include "KalmanFilter.h"

void KalmanFilter_Init(KalmanFilter *kf, float Q, float R) {
    kf->Q = Q;
    kf->R = R;
    kf->P = 1;
    kf->K = 0;
    kf->state.v = 0;
}

void KalmanFilter_Update(KalmanFilter *kf, float accel, float dt) {
    // Predict
    kf->state.v += accel * dt;
    
    // Update
    kf->P += kf->Q;
    kf->K = kf->P / (kf->P + kf->R);
    kf->state.v += kf->K * (accel - kf->state.v);
    kf->P *= (1 - kf->K);
}