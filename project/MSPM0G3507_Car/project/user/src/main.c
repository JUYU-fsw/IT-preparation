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

// 查询 HC-05 命令。返回 1=START，-1=STOP，0=无完整有效命令。
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
            car_log_write("Unknown command. Use 1=START or 0=STOP.\r\n");
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
    char uart_log[128];

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

    car_oled_init();

    // PID 测试阶段停用 MPU-6500；保留原 PA7 低电平短鸣提示。
    gpio_set_level(A7, 0);
    system_delay_ms(200);
    gpio_set_level(A7, 1);
    car_log_write("Line following ready. IMU disabled.\r\n");

    // OLED 从测试图案切换为后续开发使用的状态界面。
    car_oled_fill(0x00);
    car_oled_show_string(0, "TI CAR READY");
    car_oled_show_string(2, "IMU: DISABLED");
    car_oled_show_string(4, "MODE: LINE FOLLOW");
    car_oled_show_string(6, "SEND 1 TO START");
    car_log_write("PID gains: M1[10,0.5,0] M2[10,0.5,0].\r\n");
    car_log_write("UART1=115200, HC-05 UART3(PB2/PB3)=9600.\r\n");
    car_log_write("Gray pins: A0=PB25 A1=PB18 A2=PB21 OUT=PB22.\r\n");
    car_log_write("Send 1: wait 3 s, then follow line continuously.\r\n");
    car_log_write("Send 0 at any time for immediate motor stop.\r\n");

    motor1_encoder_previous = wheel_encoder_get_count(WHEEL_ENCODER_MOTOR1);
    motor2_encoder_previous = wheel_encoder_get_count(WHEEL_ENCODER_MOTOR2);

    while(true)
    {
        system_delay_ms(SPEED_PID_BASE_PERIOD_MS);
        control_tick ++;

        bluetooth_command = car_bluetooth_query_command();
        if(bluetooth_command < 0)
        {
            line_follow_running = 0;
            start_delay_tick = 0;
            motor1_target_rpm = 0.0f;
            motor2_target_rpm = 0.0f;
            speed_pid_set_target(&motor1_pid, 0.0f);
            speed_pid_set_target(&motor2_pid, 0.0f);
            tb6612_stop_all();
            car_log_write("ACK STOP: motors stopped.\r\n");
        }
        else if(bluetooth_command > 0)
        {
            line_follow_running = 1;
            start_delay_tick = 0;
            motor1_target_rpm = 0.0f;
            motor2_target_rpm = 0.0f;
            speed_pid_set_target(&motor1_pid, 0.0f);
            speed_pid_set_target(&motor2_pid, 0.0f);
            motor1_encoder_previous = wheel_encoder_get_count(WHEEL_ENCODER_MOTOR1);
            motor2_encoder_previous = wheel_encoder_get_count(WHEEL_ENCODER_MOTOR2);
            tb6612_stop_all();
            car_log_write("ACK START: 3 s safety delay.\r\n");
        }

        line_sensor_read(&line_sensor);

        if(line_follow_running && (start_delay_tick < 300))
        {
            start_delay_tick ++;
        }

        // 收到 START 后先等待 3 秒；巡线期间丢线立即把双轮目标清零。
        if(!line_follow_running || (start_delay_tick < 300))
        {
            motor1_target_rpm = 0.0f;
            motor2_target_rpm = 0.0f;
        }
        else
        {
            line_follow_update(&line_follow, &line_sensor,
                               &motor1_target_rpm, &motor2_target_rpm);
        }

        speed_pid_set_target(&motor1_pid, motor1_target_rpm);
        speed_pid_set_target(&motor2_pid, motor2_target_rpm);

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

        heartbeat_divider ++;
        if(50 <= heartbeat_divider)
        {
            heartbeat_divider = 0;
            gpio_toggle_level(B16);
        }

        pid_log_divider ++;
        if(100 <= pid_log_divider)
        {
            pid_log_divider = 0;
            sprintf(uart_log,
                    "LINE run=%u mode=%u mask=%02X err=%4d base=%3d corr=%4d target[L=%3d R=%3d] rpm100[L=%6ld R=%6ld]\r\n",
                    line_follow_running, (unsigned int)line_follow.mode,
                    line_sensor.mask, line_sensor.error,
                    (int)line_follow.base_rpm,
                    (int)line_follow.correction_rpm,
                    (int)motor1_target_rpm, (int)motor2_target_rpm,
                    (long)(motor1_pid.measured_rpm * 100.0f),
                    (long)(motor2_pid.measured_rpm * 100.0f));
            car_log_write(uart_log);
        }
    }
}

