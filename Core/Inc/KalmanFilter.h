typedef struct {
    float z; // Position
    float v; // Velocity
} State;

typedef struct {
    float Q; // Process noise covariance
    float R; // Measurement noise covariance
    float P; // Estimation error covariance
    float K; // Kalman gain
    State state;
} KalmanFilter;

void KalmanFilter_Init(KalmanFilter *kf, float Q, float R);
void KalmanFilter_Update(KalmanFilter *kf, float accel, float dt);