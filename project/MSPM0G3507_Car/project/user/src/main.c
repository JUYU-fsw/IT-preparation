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
// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完

// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设

// **************************** 代码区域 ****************************

static soft_iic_info_struct oled_iic;
static soft_iic_info_struct mpu6050_iic;

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

static void car_imu_read_raw (uint8 data[14],
                              int16 *accel_x, int16 *accel_y, int16 *accel_z,
                              int16 *gyro_x, int16 *gyro_y, int16 *gyro_z)
{
    soft_iic_read_8bit_registers(&mpu6050_iic, 0x3B, data, 14);
    *accel_x = (int16)(((uint16)data[0]  << 8) | data[1]);
    *accel_y = (int16)(((uint16)data[2]  << 8) | data[3]);
    *accel_z = (int16)(((uint16)data[4]  << 8) | data[5]);
    *gyro_x  = (int16)(((uint16)data[8]  << 8) | data[9]);
    *gyro_y  = (int16)(((uint16)data[10] << 8) | data[11]);
    *gyro_z  = (int16)(((uint16)data[12] << 8) | data[13]);
}

int main (void)
{
    uint8 heartbeat_divider = 0;
    uint8 imu_log_divider = 0;
    uint8 mpu6050_who_am_i = 0;
    uint8 imu_raw[14];
    uint16 calibration_sample = 0;
    int16 accel_x = 0;
    int16 accel_y = 0;
    int16 accel_z = 0;
    int16 gyro_x = 0;
    int16 gyro_y = 0;
    int16 gyro_z = 0;
    int32 accel_x_sum = 0;
    int32 accel_y_sum = 0;
    int32 accel_z_sum = 0;
    int32 gyro_x_sum = 0;
    int32 gyro_y_sum = 0;
    int32 gyro_z_sum = 0;
    int16 accel_x_bias = 0;
    int16 accel_y_bias = 0;
    int16 accel_z_bias = 0;
    int16 gyro_x_bias = 0;
    int16 gyro_y_bias = 0;
    int16 gyro_z_bias = 0;
    int32 motor1_encoder_count = 0;
    int32 motor2_encoder_count = 0;
    uint32 motor1_edge_a = 0;
    uint32 motor1_edge_b = 0;
    uint32 motor2_edge_a = 0;
    uint32 motor2_edge_b = 0;
    uint8 motor1_ab = 0;
    uint8 motor2_ab = 0;
    char uart_log[128];

    clock_init(SYSTEM_CLOCK_80M);   // 时钟配置及系统初始化<务必保留>

    // MPU6050 占用 PA0/PA1，因此调试串口迁移到载板 UART1。
    uart_init(UART_1, 115200, UART1_TX_B6, UART1_RX_B7);

    // 按终版载板测试要求，使用 PB16 作为 GPIO 心跳输出。
    // 使用翻转方式测试，不依赖外接 LED 是高电平点亮还是低电平点亮。
    gpio_init(B16, GPO, 0, GPO_PUSH_PULL);

    // 载板蜂鸣器控制信号接 PA7，模块丝印确认低电平触发。
    // 默认输出高电平，确保蜂鸣器关闭。
    gpio_init(A7, GPO, 1, GPO_PUSH_PULL);

    // 电机底层已建立，但编译期安全锁保持关闭，PWM 永远为 0。
    tb6612_init();
    wheel_encoder_init();

    car_oled_init();

    // MPU6050：AD0 接地，7 位地址为 0x68；载板 SCL=PA1、SDA=PA0。
    soft_iic_init(&mpu6050_iic, 0x68, 10, A1, A0);
    system_delay_ms(100);
    soft_iic_write_8bit_register(&mpu6050_iic, 0x6B, 0x00);  // 退出睡眠
    soft_iic_write_8bit_register(&mpu6050_iic, 0x19, 0x07);  // 1 kHz / (1 + 7) = 125 Hz
    soft_iic_write_8bit_register(&mpu6050_iic, 0x1A, 0x03);  // 数字低通滤波
    soft_iic_write_8bit_register(&mpu6050_iic, 0x1B, 0x00);  // 陀螺仪 ±250 dps
    soft_iic_write_8bit_register(&mpu6050_iic, 0x1C, 0x00);  // 加速度计 ±2 g
    soft_iic_write_8bit_register(&mpu6050_iic, 0x1D, 0x03);  // MPU-6500 加速度低通滤波
    system_delay_ms(50);

    mpu6050_who_am_i = soft_iic_read_8bit_register(&mpu6050_iic, 0x75);
    sprintf(uart_log, "IMU detected: WHO_AM_I=0x%02X (0x70=MPU-6500), I2C address=0x68.\r\n",
            mpu6050_who_am_i);
    uart_write_string(UART_1, uart_log);

    // 上电零偏校准：此阶段必须保持模块水平、静止。
    uart_write_string(UART_1, "IMU calibration: keep the car LEVEL and STILL for 4 seconds.\r\n");
    for(calibration_sample = 0; calibration_sample < 500; calibration_sample ++)
    {
        car_imu_read_raw(imu_raw, &accel_x, &accel_y, &accel_z,
                         &gyro_x, &gyro_y, &gyro_z);
        accel_x_sum += accel_x;
        accel_y_sum += accel_y;
        accel_z_sum += accel_z;
        gyro_x_sum += gyro_x;
        gyro_y_sum += gyro_y;
        gyro_z_sum += gyro_z;
        system_delay_ms(8);
    }

    accel_x_bias = (int16)(accel_x_sum / 500);
    accel_y_bias = (int16)(accel_y_sum / 500);
    accel_z_bias = (int16)((accel_z_sum / 500) - 16384);
    gyro_x_bias = (int16)(gyro_x_sum / 500);
    gyro_y_bias = (int16)(gyro_y_sum / 500);
    gyro_z_bias = (int16)(gyro_z_sum / 500);

    sprintf(uart_log, "BIAS ACC[%d,%d,%d] GYRO[%d,%d,%d]\r\n",
            accel_x_bias, accel_y_bias, accel_z_bias,
            gyro_x_bias, gyro_y_bias, gyro_z_bias);
    uart_write_string(UART_1, uart_log);

    // 校准完成提示：低电平触发蜂鸣器，短鸣一次后保持关闭。
    gpio_set_level(A7, 0);
    system_delay_ms(200);
    gpio_set_level(A7, 1);
    uart_write_string(UART_1, "IMU calibration complete.\r\n");

    // OLED 从测试图案切换为后续开发使用的状态界面。
    car_oled_fill(0x00);
    car_oled_show_string(0, "TI CAR READY");
    car_oled_show_string(2, "IMU: MPU-6500");
    car_oled_show_string(4, "CALIBRATION: OK");
    car_oled_show_string(6, "ENCODER: READY");
    uart_write_string(UART_1, "TB6612 initialized: output safety lock is ON, PWM=0.\r\n");
    uart_write_string(UART_1, "Encoder test: turn MOTOR1 and MOTOR2 wheels by hand.\r\n");

    while(true)
    {
        system_delay_ms(500);
        heartbeat_divider ++;
        if(1 <= heartbeat_divider)
        {
            heartbeat_divider = 0;
            gpio_toggle_level(B16);
        }

        imu_log_divider ++;
        if(2 <= imu_log_divider)
        {
            imu_log_divider = 0;
            // MPU-6500 数据寄存器从 0x3B 开始，按高字节在前连续读取。
            car_imu_read_raw(imu_raw, &accel_x, &accel_y, &accel_z,
                             &gyro_x, &gyro_y, &gyro_z);
            accel_x -= accel_x_bias;
            accel_y -= accel_y_bias;
            accel_z -= accel_z_bias;
            gyro_x -= gyro_x_bias;
            gyro_y -= gyro_y_bias;
            gyro_z -= gyro_z_bias;

            sprintf(uart_log, "CAL ACC[%6d,%6d,%6d] GYRO[%6d,%6d,%6d]\r\n",
                    accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z);
            uart_write_string(UART_1, uart_log);

            motor1_encoder_count = wheel_encoder_get_count(WHEEL_ENCODER_MOTOR1);
            motor2_encoder_count = wheel_encoder_get_count(WHEEL_ENCODER_MOTOR2);
            sprintf(uart_log, "ENC MOTOR1=%ld MOTOR2=%ld\r\n",
                    (long)motor1_encoder_count, (long)motor2_encoder_count);
            uart_write_string(UART_1, uart_log);

            motor1_edge_a = wheel_encoder_get_edge_count(WHEEL_ENCODER_MOTOR1, 0);
            motor1_edge_b = wheel_encoder_get_edge_count(WHEEL_ENCODER_MOTOR1, 1);
            motor2_edge_a = wheel_encoder_get_edge_count(WHEEL_ENCODER_MOTOR2, 0);
            motor2_edge_b = wheel_encoder_get_edge_count(WHEEL_ENCODER_MOTOR2, 1);
            motor1_ab = wheel_encoder_get_state(WHEEL_ENCODER_MOTOR1);
            motor2_ab = wheel_encoder_get_state(WHEEL_ENCODER_MOTOR2);
            sprintf(uart_log, "EDGE M1[A=%lu B=%lu AB=%u%u] M2[A=%lu B=%lu AB=%u%u]\r\n",
                    (unsigned long)motor1_edge_a, (unsigned long)motor1_edge_b,
                    (motor1_ab >> 1) & 1, motor1_ab & 1,
                    (unsigned long)motor2_edge_a, (unsigned long)motor2_edge_b,
                    (motor2_ab >> 1) & 1, motor2_ab & 1);
            uart_write_string(UART_1, uart_log);
        }
    }
}

