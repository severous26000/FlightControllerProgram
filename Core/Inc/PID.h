typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float previous_error;
    float integral;
} PID_t;

void PID_Init(PID_t *pid, float Kp, float Ki, float Kd);
float PID_Compute(PID_t *pid, float setpoint, float measured, float dt);