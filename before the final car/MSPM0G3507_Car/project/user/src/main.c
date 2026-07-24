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
// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完

// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设

// **************************** 代码区域 ****************************

static soft_iic_info_struct oled_iic;

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
    uint16 control_tick = 0;
    uint16 test_tick = 0;
    int32 motor1_encoder_count = 0;
    int32 motor2_encoder_count = 0;
    int32 motor1_encoder_previous = 0;
    int32 motor2_encoder_previous = 0;
    int32 motor1_encoder_delta = 0;
    int32 motor2_encoder_delta = 0;
    int16 motor1_duty = 0;
    int16 motor2_duty = 0;
    float test_target_rpm = 0.0f;
    speed_pid_struct motor1_pid;
    speed_pid_struct motor2_pid;
    char uart_log[128];

    clock_init(SYSTEM_CLOCK_80M);   // 时钟配置及系统初始化<务必保留>

    // PID 单模块测试固定使用载板 UART1。
    uart_init(UART_1, 115200, UART1_TX_B6, UART1_RX_B7);

    // 按终版载板测试要求，使用 PB16 作为 GPIO 心跳输出。
    // 使用翻转方式测试，不依赖外接 LED 是高电平点亮还是低电平点亮。
    gpio_init(B16, GPO, 0, GPO_PUSH_PULL);

    // 载板蜂鸣器控制信号接 PA7，模块丝印确认低电平触发。
    // 默认输出高电平，确保蜂鸣器关闭。
    gpio_init(A7, GPO, 1, GPO_PUSH_PULL);

    // 本副本只用于学长车架空测试，电机参数和 PID 参数均取自学长车。
    tb6612_init();
    wheel_encoder_init();
    speed_pid_init(&motor1_pid);
    speed_pid_init(&motor2_pid);

    car_oled_init();

    // MPU/IMU 代码已完全停用。蜂鸣器改为独立的 PID 测试启动提示。
    gpio_set_level(A7, 0);
    system_delay_ms(200);
    gpio_set_level(A7, 1);

    car_oled_fill(0x00);
    car_oled_show_string(0, "PID ONLY TEST");
    car_oled_show_string(2, "IMU: DISABLED");
    car_oled_show_string(4, "TARGET: 30 RPM");
    car_oled_show_string(6, "PID TEST: STANDBY");
    uart_write_string(UART_1, "PID-only test: 937.2 CPR, 50 ms control, Kp=25 Ki=1 Kd=0.\r\n");
    uart_write_string(UART_1, "Raise both wheels before the automatic forward/reverse test.\r\n");

    motor1_encoder_previous = wheel_encoder_get_count(WHEEL_ENCODER_MOTOR1);
    motor2_encoder_previous = wheel_encoder_get_count(WHEEL_ENCODER_MOTOR2);

    while(true)
    {
        system_delay_ms(SPEED_PID_BASE_PERIOD_MS);
        control_tick ++;
        test_tick ++;

        // 18 秒循环：3 秒停车、6 秒 +30 RPM、3 秒停车、6 秒 -30 RPM。
        if(test_tick < 300)
        {
            test_target_rpm = 0.0f;
        }
        else if(test_tick < 900)
        {
            test_target_rpm = 30.0f;
        }
        else if(test_tick < 1200)
        {
            test_target_rpm = 0.0f;
        }
        else if(test_tick < 1800)
        {
            test_target_rpm = -30.0f;
        }
        else
        {
            test_tick = 0;
            test_target_rpm = 0.0f;
        }

        speed_pid_set_target(&motor1_pid, test_target_rpm);
        speed_pid_set_target(&motor2_pid, test_target_rpm);

        if(0 == (control_tick % SPEED_PID_CONTROL_DIVIDER))
        {
            motor1_encoder_count = wheel_encoder_get_count(WHEEL_ENCODER_MOTOR1);
            motor2_encoder_count = wheel_encoder_get_count(WHEEL_ENCODER_MOTOR2);

            // 实测确认：相同 PWM 符号下两路编码器原始方向相反。
            motor1_encoder_delta = -(motor1_encoder_count - motor1_encoder_previous);
            motor2_encoder_delta =  (motor2_encoder_count - motor2_encoder_previous);
            motor1_encoder_previous = motor1_encoder_count;
            motor2_encoder_previous = motor2_encoder_count;

            motor1_duty = speed_pid_update(&motor1_pid, motor1_encoder_delta);
            motor2_duty = speed_pid_update(&motor2_pid, motor2_encoder_delta);
            tb6612_set_motor(TB6612_MOTOR_A, motor1_duty);
            tb6612_set_motor(TB6612_MOTOR_B, motor2_duty);
        }

        if(0 == (control_tick % 50))
        {
            gpio_toggle_level(B16);
            sprintf(uart_log,
                    "PID T=%4u target=%4d M1[rpm_x100=%6ld duty=%5d d50=%ld] M2[rpm_x100=%6ld duty=%5d d50=%ld]\r\n",
                    control_tick, (int)test_target_rpm,
                    (long)(motor1_pid.measured_rpm * 100.0f),
                    motor1_duty, (long)motor1_encoder_delta,
                    (long)(motor2_pid.measured_rpm * 100.0f),
                    motor2_duty, (long)motor2_encoder_delta);
            uart_write_string(UART_1, uart_log);
        }
    }
}

