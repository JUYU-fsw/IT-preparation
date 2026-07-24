#ifndef _tb6612_h_
#define _tb6612_h_

#include "zf_common_typedef.h"

typedef enum
{
    TB6612_MOTOR_A = 0,
    TB6612_MOTOR_B,
} tb6612_motor_enum;

// 本副本专用于学长车架空 PID 测试；原工程的安全锁仍保持不变。
#define TB6612_OUTPUT_ENABLE    (1)

#define TB6612_DUTY_MAX         (10000)
#define TB6612_PWM_FREQUENCY    (20000)

void tb6612_init       (void);
void tb6612_set_motor  (tb6612_motor_enum motor, int16 duty);
void tb6612_stop_motor (tb6612_motor_enum motor);
void tb6612_stop_all   (void);

#endif
