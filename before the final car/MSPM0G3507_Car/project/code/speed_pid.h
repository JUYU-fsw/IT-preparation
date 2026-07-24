#ifndef _speed_pid_h_
#define _speed_pid_h_

#include "zf_common_typedef.h"

// 学长车电机参数：11 PPR，AB 四倍频，21.3:1 减速。
#define SENIOR_MOTOR_ENCODER_PPR          (11.0f)
#define SENIOR_MOTOR_ENCODER_MULTIPLIER   (4.0f)
#define SENIOR_MOTOR_GEAR_RATIO           (21.3f)
#define SENIOR_MOTOR_COUNTS_PER_REV       (937.2f)

#define SPEED_PID_BASE_PERIOD_MS          (10)
#define SPEED_PID_PERIOD_MS               (50)
#define SPEED_PID_CONTROL_DIVIDER         (SPEED_PID_PERIOD_MS / SPEED_PID_BASE_PERIOD_MS)

// 学长工程的 35/10 增益对应另一套速度单位和实现。本工程直接使用 RPM，
// 首轮实测出现 ±3000 饱和振荡，因此从保守增益重新整定。
#define SPEED_PID_KP                      (10.0f)
#define SPEED_PID_KI                      (0.0f)
#define SPEED_PID_KD                      (0.0f)
#define SPEED_PID_OUTPUT_LIMIT            (3000.0f)
#define SPEED_PID_START_DUTY              (1000.0f)
#define SPEED_PID_FILTER_ALPHA             (0.35f)

typedef struct
{
    float target_rpm;
    float raw_rpm;
    float measured_rpm;
    float output;
    float error_1;
    float error_2;
} speed_pid_struct;

void  speed_pid_init       (speed_pid_struct *pid);
void  speed_pid_reset      (speed_pid_struct *pid);
void  speed_pid_set_target (speed_pid_struct *pid, float target_rpm);
int16 speed_pid_update     (speed_pid_struct *pid, int32 encoder_delta);

#endif
