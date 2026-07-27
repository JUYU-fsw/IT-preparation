/*********************************************************************************************************************
* MSPM0G3507 Opensource Library 即（MSPM0G3507 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
* 
* 本文件是 MSPM0G3507 开源库的一部分
* 
* MSPM0G3507 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
* 
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
* 
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
* 
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
* 
* 文件名称          mian
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          MDK 5.37
* 适用平台          MSPM0G3507
* 店铺链接          https://seekfree.taobao.com/
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "tb6612.h"
#include "wheel_encoder.h"
#include "speed_pid.h"
#include "line_sensor.h"
#include "line_follow.h"
#include "mpu6050_yaw.h"
#include "angle_pid.h"
#include "odometer.h"
// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完

// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设

// **************************** 代码区域 ****************************

static soft_iic_info_struct oled_iic;

// UART1 保留有线调试，UART3(PB2/PB3)连接板载 UART4 接口上的 HC-05。
static void car_log_write (const char *text)
{
    uart_write_string(UART_1, text);
    uart_write_string(UART_3, text);
}

/* 1 Hz compact telemetry is sent to both UART1 and the HC-05. */
static void car_debug_write (const char *text)
{
    uart_write_string(UART_1, text);
    uart_write_string(UART_3, text);
}

// 查询 HC-05 命令。
// 返回 1=START, -1=STOP, 2=RESET_YAW, 3=ODO_QUERY,
//      4=STATIONARY_HOLD, 5=HELP。
static int8 car_bluetooth_query_command (void)
{
    static char command[16];
    static uint8 command_length = 0;
    uint8 data;

    while(uart_query_byte(UART_3, &data))
    {
        // 单字节命令优先处理，避免低速串口连续字符串超过接收 FIFO。
        if('1' == data)
        {
            command_length = 0;
            return 1;
        }
        if('0' == data)
        {
            command_length = 0;
            return -1;
        }
        if(('h' == data) || ('H' == data))
        {
            command_length = 0;
            return 4;
        }
        if('r' == data || 'R' == data)
        {
            command_length = 0;
            return 2;
        }
        if('d' == data || 'D' == data)
        {
            command_length = 0;
            return 3;
        }
        if('?' == data)
        {
            command_length = 0;
            return 5;
        }

        if(('\r' == data) || ('\n' == data))
        {
            if(0 == command_length)
            {
                continue;
            }

            command[command_length] = '\0';
            command_length = 0;
            if(0 == strcmp(command, "START"))
            {
                return 1;
            }
            if(0 == strcmp(command, "STOP"))
            {
                return -1;
            }
            if(0 == strcmp(command, "RESET"))
            {
                return 2;
            }
            if(0 == strcmp(command, "DIST"))
            {
                return 3;
            }
            if(0 == strcmp(command, "HELP"))
            {
                return 5;
            }
            car_log_write("ERR CMD; send ?\r\n");
        }
        else if(command_length < (sizeof(command) - 1))
        {
            if((data >= 'a') && (data <= 'z'))
            {
                data = (uint8)(data - 'a' + 'A');
            }
            command[command_length] = (char)data;
            command_length ++;
            command[command_length] = '\0';

            // 蓝牙终端可能不发送回车；收到完整关键字后立即执行。
            if(0 == strcmp(command, "START"))
            {
                command_length = 0;
                return 1;
            }
            if(0 == strcmp(command, "STOP"))
            {
                command_length = 0;
                return -1;
            }
            if(0 == strcmp(command, "RESET"))
            {
                command_length = 0;
                return 2;
            }
            if(0 == strcmp(command, "DIST"))
            {
                command_length = 0;
                return 3;
            }
            if(0 == strcmp(command, "HELP"))
            {
                command_length = 0;
                return 5;
            }
        }
        else
        {
            command_length = 0;
        }
    }
    return 0;
}

// SSD1306 I2C 控制字：0x00 表示后续字节为命令。
static void car_oled_write_command (uint8 command)
{
    uint8 packet[2] = {0x00, command};
    soft_iic_write_8bit_array(&oled_iic, packet, 2);
}

// 向 128x64 SSD1306 写满 1024 字节显存。
static void car_oled_fill (uint8 pattern)
{
    uint16 i;

    car_oled_write_command(0x21);    // 设置列地址
    car_oled_write_command(0x00);
    car_oled_write_command(0x7F);
    car_oled_write_command(0x22);    // 设置页地址
    car_oled_write_command(0x00);
    car_oled_write_command(0x07);

    soft_iic_start(&oled_iic);
    soft_iic_send_data(&oled_iic, oled_iic.addr << 1);
    soft_iic_send_data(&oled_iic, 0x40);       // 后续字节为显存数据
    for(i = 0; i < 1024; i ++)
    {
        soft_iic_send_data(&oled_iic, pattern);
    }
    soft_iic_stop(&oled_iic);
}

static void car_oled_show_string (uint8 page, const char *text)
{
    uint8 column = 0;
    uint8 font_index = 0;
    uint8 font_column = 0;

    car_oled_write_command(0x21);
    car_oled_write_command(0x00);
    car_oled_write_command(0x7F);
    car_oled_write_command(0x22);
    car_oled_write_command(page);
    car_oled_write_command(page);

    soft_iic_start(&oled_iic);
    soft_iic_send_data(&oled_iic, oled_iic.addr << 1);
    soft_iic_send_data(&oled_iic, 0x40);

    while(('\0' != *text) && (column <= 121))
    {
        if((*text < 32) || (*text > 126))
        {
            font_index = 0;
        }
        else
        {
            font_index = (uint8)(*text - 32);
        }

        for(font_column = 0; font_column < 6; font_column ++)
        {
            soft_iic_send_data(&oled_iic, ascii_font_6x8[font_index][font_column]);
            column ++;
        }
        text ++;
    }

    while(column < 128)
    {
        soft_iic_send_data(&oled_iic, 0x00);
        column ++;
    }
    soft_iic_stop(&oled_iic);
}

static void car_oled_init (void)
{
    // 载板 OLED：SCL=PA31，SDA=PA28；常见 SSD1306 地址为 0x3C。
    soft_iic_init(&oled_iic, 0x3C, 10, A31, A28);
    system_delay_ms(100);

    car_oled_write_command(0xAE);    // 关闭显示
    car_oled_write_command(0xD5);
    car_oled_write_command(0x80);
    car_oled_write_command(0xA8);
    car_oled_write_command(0x3F);
    car_oled_write_command(0xD3);
    car_oled_write_command(0x00);
    car_oled_write_command(0x40);
    car_oled_write_command(0x8D);
    car_oled_write_command(0x14);
    car_oled_write_command(0x20);
    car_oled_write_command(0x00);    // 水平寻址模式
    car_oled_write_command(0xA1);
    car_oled_write_command(0xC8);
    car_oled_write_command(0xDA);
    car_oled_write_command(0x12);
    car_oled_write_command(0x81);
    car_oled_write_command(0x7F);
    car_oled_write_command(0xD9);
    car_oled_write_command(0xF1);
    car_oled_write_command(0xDB);
    car_oled_write_command(0x40);
    car_oled_write_command(0xA4);
    car_oled_write_command(0xA6);
    car_oled_write_command(0xAF);    // 开启显示

    car_oled_fill(0xAA);
}

int main (void)
{
    uint8 heartbeat_divider = 0;
    uint8 pid_log_divider = 0;
    uint8 line_follow_running = 0;
    int8 bluetooth_command = 0;
    uint16 control_tick = 0;
    uint16 start_delay_tick = 0;
    int32 motor1_encoder_count = 0;
    int32 motor2_encoder_count = 0;
    int32 motor1_encoder_previous = 0;
    int32 motor2_encoder_previous = 0;
    int32 motor1_encoder_delta = 0;
    int32 motor2_encoder_delta = 0;
    int16 motor1_duty = 0;
    int16 motor2_duty = 0;
    float motor1_target_rpm = 0.0f;
    float motor2_target_rpm = 0.0f;
    speed_pid_struct motor1_pid;
    speed_pid_struct motor2_pid;
    line_sensor_data_struct line_sensor;
    line_follow_struct line_follow;
    angle_pid_struct angle_pid;
    odometer_struct  odo;
    float yaw_angle        = 0.0f;
    float yaw_target       = 0.0f;
    float angle_diff_rpm   = 0.0f;
    float mount_axis_x     = 0.0f;
    float mount_axis_y     = 0.0f;
    float mount_axis_z     = 1.0f;
    uint8 stationary_hold_active = 0;
    /*
     * Detailed IMU telemetry is longer than 128 bytes.  The old buffer
     * overflowed during sprintf and could corrupt Bluetooth/parser state.
     */
    char uart_log[256];

    clock_init(SYSTEM_CLOCK_80M);   // 时钟配置及系统初始化<务必保留>

    // UART1：有线调试；UART3：板载 UART4 接口，连接 HC-05。
    uart_init(UART_1, 115200, UART1_TX_B6, UART1_RX_B7);
    uart_init(UART_3, 9600, UART3_TX_B2, UART3_RX_B3);

    // 按终版载板测试要求，使用 PB16 作为 GPIO 心跳输出。
    // 使用翻转方式测试，不依赖外接 LED 是高电平点亮还是低电平点亮。
    gpio_init(B16, GPO, 0, GPO_PUSH_PULL);

    // 载板蜂鸣器控制信号接 PA7，模块丝印确认低电平触发。
    // 默认输出高电平，确保蜂鸣器关闭。
    gpio_init(A7, GPO, 1, GPO_PUSH_PULL);

    // 外层工程切换为自有电机，速度换算固定采用商家给出的 2450 count/rev。
    tb6612_init();
    wheel_encoder_init();
    speed_pid_init(&motor1_pid);
    speed_pid_init(&motor2_pid);
    line_sensor_init();
    line_follow_init(&line_follow);
    angle_pid_init(&angle_pid);
    odometer_init(&odo);

    car_oled_init();

    // ---- IMU init & calibration (vehicle must be stationary; tilt is allowed) ----
    car_oled_fill(0x00);
    car_oled_show_string(0, "CALIBRATING IMU");
    car_oled_show_string(4, "KEEP STILL...");
    car_log_write("MPU6050/6500 init...\r\n");
    mpu6050_yaw_init();
    system_delay_ms(100);
    mpu6050_yaw_calibrate(MPU6500_CALIB_SAMPLES);
    if(mpu6050_yaw_is_ready())
    {
        car_log_write("MPU6050/6500 calibration done.\r\n");
    }
    else
    {
        car_log_write("ERROR IMU NOT FOUND: check 5V/GND/PA1-SCL/PA0-SDA.\r\n");
        car_oled_show_string(2, "IMU: NOT FOUND");
        car_oled_show_string(4, "CHECK WIRING");
    }
    mpu6050_yaw_get_mount_axis(&mount_axis_x, &mount_axis_y, &mount_axis_z);
    sprintf(uart_log, "IMU_AXIS x=%+.4f y=%+.4f z=%+.4f\r\n",
            (double)mount_axis_x, (double)mount_axis_y, (double)mount_axis_z);
    car_log_write(uart_log);

    // 校准完成后短鸣
    gpio_set_level(A7, 0);
    system_delay_ms(200);
    gpio_set_level(A7, 1);

    // 锁定当前航向为目标（0度）
    yaw_target      = 0.0f;
    angle_pid_set_target(&angle_pid, yaw_target);

    // ---- startup status on OLED ----
    car_oled_fill(0x00);
    if(mpu6050_yaw_is_ready())
    {
        car_oled_show_string(0, "TI CAR v2 IMU ON");
        car_oled_show_string(2, "YAW:   0.0 deg");
        car_oled_show_string(4, "ANGL: MON  ");
        car_oled_show_string(6, "SEND 1 TO START");
        car_log_write("Angle-loop car ready. MPU6050/6500 yaw active.\r\n");
        car_log_write("Yaw scale=1.000; recalibrate for the new IMU.\r\n");
    }
    else
    {
        car_oled_show_string(0, "TI CAR IMU ERROR");
        car_oled_show_string(2, "WHO: NO RESPONSE");
        car_oled_show_string(4, "HOLD DISABLED");
        car_oled_show_string(6, "CHECK MPU WIRES");
        car_log_write("Car ready with IMU fault; heading hold disabled.\r\n");
    }
    car_log_write("PID: speed[10,1,0]; line/angle separated.\r\n");
    car_log_write("UART1=115200, HC-05 UART3(PB2/PB3)=9600.\r\n");
    car_log_write("CMD:1 0 h r d ?\r\n");

    motor1_encoder_previous = wheel_encoder_get_count(WHEEL_ENCODER_MOTOR1);
    motor2_encoder_previous = wheel_encoder_get_count(WHEEL_ENCODER_MOTOR2);

    while(true)
    {
        system_delay_ms(SPEED_PID_BASE_PERIOD_MS);
        control_tick ++;

        /* ---- bluetooth command dispatch ---- */
        bluetooth_command = car_bluetooth_query_command();
        if(bluetooth_command < 0)
        {
            line_follow_running = 0;
            start_delay_tick    = 0;
            motor1_target_rpm   = 0.0f;
            motor2_target_rpm   = 0.0f;
            angle_diff_rpm      = 0.0f;
            stationary_hold_active = 0;
            angle_pid_reset(&angle_pid);
            speed_pid_reset(&motor1_pid);
            speed_pid_reset(&motor2_pid);
            tb6612_stop_all();
            car_log_write("OK STOP\r\n");
        }
        else if(1 == bluetooth_command)
        {
            line_follow_running = 1;
            start_delay_tick    = 0;
            motor1_target_rpm   = 0.0f;
            motor2_target_rpm   = 0.0f;
            angle_diff_rpm      = 0.0f;
            stationary_hold_active = 0;
            yaw_target = mpu6050_yaw_get_angle();  /* lock current heading */
            angle_pid_set_target(&angle_pid, yaw_target);
            speed_pid_reset(&motor1_pid);
            speed_pid_reset(&motor2_pid);
            motor1_encoder_previous = wheel_encoder_get_count(WHEEL_ENCODER_MOTOR1);
            motor2_encoder_previous = wheel_encoder_get_count(WHEEL_ENCODER_MOTOR2);
            tb6612_stop_all();
            car_log_write("OK START 3S\r\n");
        }
        else if(2 == bluetooth_command)
        {
            /*
             * A plain angle reset does not remove gyro temperature drift.
             * Stop first, then obtain a fresh stationary bias calibration.
             */
            line_follow_running = 0;
            start_delay_tick = 0;
            stationary_hold_active = 0;
            motor1_target_rpm = 0.0f;
            motor2_target_rpm = 0.0f;
            speed_pid_reset(&motor1_pid);
            speed_pid_reset(&motor2_pid);
            tb6612_stop_all();
            car_log_write("CAL KEEP STILL\r\n");
            mpu6050_yaw_calibrate(MPU6500_CALIB_SAMPLES);
            mpu6050_yaw_set_angle(0.0f);
            yaw_target = 0.0f;
            angle_pid_reset(&angle_pid);
            angle_pid_set_target(&angle_pid, yaw_target);
            car_log_write("OK CAL YAW=0\r\n");
        }
        else if(3 == bluetooth_command)
        {
            /* query odometer */
            sprintf(uart_log, "ODO total=%.1f cm left=%.1f right=%.1f\r\n",
                    (double)odometer_get_cm(&odo),
                    (double)odo.left_cm, (double)odo.right_cm);
            car_log_write(uart_log);
        }
        else if(4 == bluetooth_command)
        {
            /*
             * Explicit low-speed commissioning mode.  Lock the current
             * heading; STOP remains an unconditional emergency exit.
             */
            if(!mpu6050_yaw_is_ready())
            {
                stationary_hold_active = 0;
                tb6612_stop_all();
                car_log_write("ERR HOLD IMU\r\n");
                continue;
            }
            line_follow_running = 0;
            start_delay_tick = 0;
            stationary_hold_active = 1;
            yaw_target = mpu6050_yaw_get_angle();
            angle_pid_reset(&angle_pid);
            angle_pid_set_target(&angle_pid, yaw_target);
            car_log_write("OK HOLD; 0=EXIT\r\n");
        }
        else if(5 == bluetooth_command)
        {
            car_log_write("CMD:1 0 h r d ?\r\n");
        }

        /* ---- sensors ---- */
        line_sensor_read(&line_sensor);
        /*
         * While parked, use the full accel+gyro update so the stationary
         * detector can track temperature-dependent gyro bias. During motor
         * control use the shorter gyro-only read to keep the loop periodic.
         */
        if(!line_follow_running && !stationary_hold_active)
        {
            mpu6050_yaw_update();
        }
        else
        {
            mpu6050_yaw_update_fast();
        }
        yaw_angle = mpu6050_yaw_get_angle();

        /* ---- 3 s safety delay ---- */
        if(line_follow_running && (start_delay_tick < 300))
        {
            start_delay_tick ++;
        }

        /* ---- compute RPM targets ---- */
        if(stationary_hold_active)
        {
            float hold_error;

            angle_pid_set_target(&angle_pid, yaw_target);
            angle_diff_rpm = angle_pid_update(&angle_pid, yaw_angle,
                ANGLE_PID_HOLD_KP, ANGLE_PID_HOLD_KI, ANGLE_PID_HOLD_KD,
                ANGLE_PID_HOLD_OUTPUT_MAX, ANGLE_PID_HOLD_INTEGRAL_MAX,
                0.0127f);
            hold_error = angle_pid.error;

            /* A dead band prevents gearbox chatter around the target. */
            if((hold_error > -ANGLE_PID_HOLD_DEADBAND_DEG)
               && (hold_error < ANGLE_PID_HOLD_DEADBAND_DEG))
            {
                angle_diff_rpm = 0.0f;
                angle_pid_reset(&angle_pid);
            }

            motor1_target_rpm = -angle_diff_rpm;
            motor2_target_rpm =  angle_diff_rpm;
        }
        else if(!line_follow_running || (start_delay_tick < 300))
        {
            motor1_target_rpm = 0.0f;
            motor2_target_rpm = 0.0f;
            angle_diff_rpm    = 0.0f;
        }
        else
        {
            /*
             * Line following exclusively owns both wheel targets.
             * The former weak/strong yaw overlay fought the line controller
             * and caused rapid left/right corrections on the track.
             * Angle PID remains available only in explicit HOLD mode.
             */
            line_follow_update(&line_follow, &line_sensor,
                               &motor1_target_rpm, &motor2_target_rpm);
            angle_diff_rpm = 0.0f;
        }

        speed_pid_set_target(&motor1_pid, motor1_target_rpm);
        speed_pid_set_target(&motor2_pid, motor2_target_rpm);

        /* ---- speed PI every 20 ms ---- */
        if(0 == (control_tick % SPEED_PID_CONTROL_DIVIDER))
        {
            motor1_encoder_count = wheel_encoder_get_count(WHEEL_ENCODER_MOTOR1);
            motor2_encoder_count = wheel_encoder_get_count(WHEEL_ENCODER_MOTOR2);

            motor1_encoder_delta = SPEED_PID_MOTOR1_ENCODER_SIGN
                                 * (motor1_encoder_count - motor1_encoder_previous);
            motor2_encoder_delta = SPEED_PID_MOTOR2_ENCODER_SIGN
                                 * (motor2_encoder_count - motor2_encoder_previous);
            motor1_encoder_previous = motor1_encoder_count;
            motor2_encoder_previous = motor2_encoder_count;

            /* odometer: always accumulates */
            odometer_update(&odo, motor1_encoder_delta, motor2_encoder_delta);

            motor1_duty = speed_pid_update(&motor1_pid, motor1_encoder_delta,
                                           SPEED_PID_MOTOR1_KP,
                                           SPEED_PID_MOTOR1_KI,
                                           SPEED_PID_MOTOR1_KD);
            motor2_duty = speed_pid_update(&motor2_pid, motor2_encoder_delta,
                                           SPEED_PID_MOTOR2_KP,
                                           SPEED_PID_MOTOR2_KI,
                                           SPEED_PID_MOTOR2_KD);
            // 右轮：TB6612 A + MOTOR2 编码器；左轮：TB6612 B + MOTOR1(PA25/PA14)。
            tb6612_set_motor(TB6612_MOTOR_A,
                             SPEED_PID_TB6612_A_FORWARD_SIGN * motor2_duty);
            tb6612_set_motor(TB6612_MOTOR_B,
                             SPEED_PID_TB6612_B_FORWARD_SIGN * motor1_duty);
        }

        /* ---- heartbeat LED (500 ms) ---- */
        heartbeat_divider ++;
        if(50 <= heartbeat_divider)
        {
            heartbeat_divider = 0;
            gpio_toggle_level(B16);

            /* OLED refresh every 500 ms */
            {
                sprintf(uart_log, "L%+03d R%+03d",
                        (int)motor1_target_rpm, (int)motor2_target_rpm);
                car_oled_show_string(0, uart_log);
                sprintf(uart_log, "YAW: %+6.1f", (double)yaw_angle);
                car_oled_show_string(2, uart_log);
                if (stationary_hold_active)
                {
                    car_oled_show_string(4, "ANGL: PARK ");
                }
                else if (line_follow_running)
                {
                    car_oled_show_string(4, "ANGL: MON   ");
                }
                else
                {
                    car_oled_show_string(4, "ANGL: OFF   ");
                }
                sprintf(uart_log, "R%u D%03u M%02X",
                        line_follow_running,
                        (unsigned int)(start_delay_tick / 10),
                        line_sensor.mask);
                car_oled_show_string(6, uart_log);
            }
        }

        /*
         * 1 Hz Bluetooth log only while the motors are inactive.
         * At 9600 baud a long blocking line pauses the control loop for
         * roughly 100 ms, corrupting both speed control and yaw integration.
         * Send STOP after a test and copy the resumed stationary logs.
         */
        pid_log_divider ++;
        if(100 <= pid_log_divider)
        {
            pid_log_divider = 0;
            if(!line_follow_running && !stationary_hold_active)
            {
                sprintf(uart_log,
                        "T w=%02X y=%+.1f r=%+.2f m=%02X e=%d odo=%.1f\r\n",
                        mpu6050_yaw_read_who_am_i(),
                        (double)yaw_angle,
                        (double)mpu6050_yaw_get_rate_dps(),
                        line_sensor.mask,
                        line_sensor.error,
                        (double)odometer_get_cm(&odo));
                car_debug_write(uart_log);
            }
        }
    }
}

