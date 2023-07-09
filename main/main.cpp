/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <cstdio>
#include <cstring>
#include <cstdlib>

#include "MPU6886.h"
#include "rotor.h"

#define LGFX_M5ATOMS3
#include <LovyanGFX.hpp>
#include <LGFX_AUTODETECT.hpp>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/i2c.h"
#include "driver/gpio.h"

// Button
#define BUTTON_PIN GPIO_NUM_41

// I2C:MPU6886
#define I2C_MASTER_SCL_IO               (gpio_num_t)39
#define I2C_MASTER_SDA_IO               (gpio_num_t)38
#define I2C_MASTER_NUM                  (i2c_port_t)I2C_NUM_0
#define I2C_MASTER_TX_BUF_DISABLE       0
#define I2C_MASTER_RX_BUF_DISABLE       0
#define I2C_MASTER_FREQ_HZ              400000

#define I2C_PORT                        I2C_MASTER_NUM
#define WRITE_BIT                       I2C_MASTER_WRITE
#define READ_BIT                        I2C_MASTER_READ
#define ACK_CHECK_EN                    0x1
#define ACK_CHECK_DIS                   0x0
#define ACK_VAL                         (i2c_ack_type_t)0x0
#define NACK_VAL                        (i2c_ack_type_t)0x1

void _I2CInit(void)
{
    i2c_port_t i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode,
                       I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE,
                       0);
}

esp_err_t _I2CReadN(uint8_t addr, uint8_t reg, uint8_t *buf, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1)|WRITE_BIT,
			  ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, 1000/portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return ret;
    }
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1)|READ_BIT,
			  ACK_CHECK_EN);
    if (len > 1) {
        i2c_master_read(cmd, buf, len - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, buf + len - 1, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_PORT, cmd, 1000/portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t _I2CWrite1(uint8_t addr, uint8_t reg, uint8_t val)
{
    uint8_t d = val;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1)|WRITE_BIT,
			  ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_write(cmd, &d, 1, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, 1000/portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// This sample application displays AHRS result.

#define GRAVITY_MSS     9.80665f

// For gyro
// 10 second
#define IN_CALIB 2000
#define CALIB_ALPHA 0.002f

float c, si, sj, sk;

void imu_task(void *arg)
{
    float gx, gy, gz;
    float ax, ay, az;
    float gx_offs = 0, gy_offs = 0, gz_offs = 0;
    int count = 0;

    vTaskDelay(100/portTICK_PERIOD_MS);
    _I2CInit();
    vTaskDelay(100/portTICK_PERIOD_MS);

    MPU6886 IMU;

    IMU.Init();

    // Sample rate 200Hz. RI is updated in every 5ms.
    RotorIyVS RI (0.01, 5.0e-3, 1.0e-6);

    for (;;) {
        if (!IMU.dataReady()) {
            vTaskDelay(1/portTICK_PERIOD_MS);
            continue;
        }
        IMU.getAccelData(&ax, &ay, &az);
        IMU.getGyroData(&gx, &gy, &gz);
        ax *= GRAVITY_MSS;
        ay *= GRAVITY_MSS;
        az *= GRAVITY_MSS;
        //printf("ax: %f ay: %f az: %f\n", ax, ay, az);
        //printf("gx: %f gy: %f gz: %f\n", gx, gy, gz);

        // simple gyro offset calibration
        if (count < IN_CALIB) {
            gx_offs = (1-CALIB_ALPHA)*gx_offs + CALIB_ALPHA*gx;
            gy_offs = (1-CALIB_ALPHA)*gy_offs + CALIB_ALPHA*gy;
            gz_offs = (1-CALIB_ALPHA)*gz_offs + CALIB_ALPHA*gz;
            count++;
            continue;
	    }

        gx -= gx_offs; gy -= gy_offs; gz -= gz_offs;
        //printf("ax: %f ay: %f az: %f\n", ax, ay, az);
        //printf("gx: %f gy: %f gz: %f\n", gx, gy, gz);

        // Convert NED frame into the normal frame.
        // Since gz will be used as the coefficient of bivector
        // e1^e2, e1 <-> e2 reverses the sign of gz.
        float oi, oj, ok;
        RI.Update (gy, gx, -gz, -ay, -ax, az,
                   c, si, sj, sk, oi, oj, ok);
        //printf("%4.2f ", (float)(count - IN_CALIB)/200);
        if (count % 40 == 0)
            RI.Show ();

        count++;
    }
}

// v = S*(-e1)/S
inline static void
apply_me1 (float c, float si, float sj, float sk,
	   float& vx, float& vy, float& vz)
{
  vx = -c*c - si*si + sj*sj + sk*sk;
  vy = -2*c*sk - 2*si*sj;
  vz = 2*c*sj - 2*si*sk;
}

// v = S*(-e2)/S
inline static void
apply_me2 (float c, float si, float sj, float sk,
	   float& vx, float& vy, float& vz)
{
  vx = 2*c*sk - 2*si*sj;
  vy = -c*c + si*si - sj*sj + sk*sk;
  vz = -2*c*si - 2*sj*sk;
}

// v = S*(-e3)/S
inline static void
apply_me3 (float c, float si, float sj, float sk,
	   float& vx, float& vy, float& vz)
{
  vx = -2*c*sj - 2*si*sk;
  vy =  2*c*si - 2*sj*sk;
  vz = -c*c + si*si + sj*sj - sk*sk;
}

static LGFX lcd;

void lcd_task(void *arg)
{
    float ex[3], ey[3], ez[3];
    ex[0] = 1.0; ex[1] = 0.0; ex[2] = 0.0;
    ey[0] = 0.0; ey[1] = 1.0; ey[2] = 0.0;
    ez[0] = 0.0; ez[1] = 0.0; ez[2] = 1.0;

    while (true) {
        vTaskDelay(10 / portTICK_PERIOD_MS);

        const int R = 48;
        const int CX = 64;
        const int CY = 64;

        lcd.drawLine(CX,CY,CX+int(R*ex[0]),CY+int(R*ex[1]), 0);
        lcd.drawLine(CX,CY,CX+int(R*ey[0]),CY+int(R*ey[1]), 0);
        lcd.drawLine(CX,CY,CX+int(R*ez[0]),CY+int(R*ez[1]), 0);
        apply_me1 (c, si, sj, sk, ex[0], ex[1], ex[2]);
        apply_me2 (c, si, sj, sk, ey[0], ey[1], ey[2]);
        apply_me3 (c, si, sj, sk, ez[0], ez[1], ez[2]);
        lcd.drawLine(CX,CY,CX+int(R*ex[0]),CY+int(R*ex[1]), lcd.color888(255,0,0));
        lcd.drawLine(CX,CY,CX+int(R*ey[0]),CY+int(R*ey[1]), lcd.color888(0,255,0));
        lcd.drawLine(CX,CY,CX+int(R*ez[0]),CY+int(R*ez[1]), lcd.color888(0,0,255));
    }
}

extern "C" void app_main()
{

    lcd.init();
    lcd.setRotation(1);
    lcd.setBrightness(128);
    lcd.setColorDepth(24);
    
    xTaskCreate(imu_task, "imu_task", 8192, NULL, 1, NULL);
    xTaskCreate(lcd_task, "lcd_task", 8192, NULL, 2, NULL);

    gpio_set_direction(BUTTON_PIN, GPIO_MODE_INPUT);
    while (true) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
