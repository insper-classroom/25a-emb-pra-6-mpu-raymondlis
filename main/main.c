#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>

#include "pico/stdlib.h"
#include <stdio.h>
#include <stdlib.h>

#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "mpu6050.h"
#include "Fusion.h"

#define UART_ID           uart0
#define BAUD_RATE         115200
#define UART_TX_PIN       0
#define UART_RX_PIN       1

#define SAMPLE_MS         10
#define SAMPLE_PERIOD     (SAMPLE_MS/1000.0f)

#define K_ROLL            2.0f
#define K_PITCH           2.0f
#define CLICK_THRESHOLD   24576

const int MPU_ADDRESS = 0x68;
const int I2C_SDA_GPIO = 4;
const int I2C_SCL_GPIO = 5;

typedef struct {
    uint8_t axis;     // 0 = roll→X, 1 = pitch→Y, 2 = click
    int16_t val;      // -255..+255 para movimento, 1 para click
} event_t;

static QueueHandle_t xQueuePos;

static void mpu6050_reset() {
    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in different ways that could be added here
    uint8_t buf[] = {0x6B, 0x00};
    i2c_write_blocking(i2c_default, MPU_ADDRESS, buf, 2, false);
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    // For this particular device, we send the device the register we want to read
    // first, then subsequently read from the device. The register is auto incrementing
    // so we don't need to keep sending the register we want, just the first.

    uint8_t buffer[6];

    // Start reading acceleration registers from register 0x3B for 6 bytes
    uint8_t val = 0x3B;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true); // true to keep master control of bus
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);

    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Now gyro data from reg 0x43 for 6 bytes
    // The register is auto incrementing on each read
    val = 0x43;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);  // False - finished with bus

    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);;
    }

    // Now temperature from reg 0x41 for 2 bytes
    // The register is auto incrementing on each read
    val = 0x41;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 2, false);  // False - finished with bus

    *temp = buffer[0] << 8 | buffer[1];
}

void mpu6050_task(void *p) {
    // configuracao do I2C
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(I2C_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_GPIO);
    gpio_pull_up(I2C_SCL_GPIO);

    mpu6050_reset();
    
    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);
    
    int16_t acceleration[3], gyro[3], temp;
    float last_roll = 0, last_pitch = 0;
    bool clicked = false;

    while(1) {
        // leitura da MPU, sem fusao de dados
        mpu6050_read_raw(acceleration, gyro, &temp);

        FusionVector g = {
            .axis.x = gyro[0]/131.0f,
            .axis.y = gyro[1]/131.0f,
            .axis.z = gyro[2]/131.0f
        };
        FusionVector a = {
            .axis.x = acceleration[0]/16384.0f,
            .axis.y = acceleration[1]/16384.0f,
            .axis.z = acceleration[2]/16384.0f
        };

        FusionAhrsUpdateNoMagnetometer(&ahrs, g, a, SAMPLE_PERIOD);
        FusionEuler e = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

        float dr = e.angle.roll  - last_roll;
        float dp = e.angle.pitch - last_pitch;
        last_roll  = e.angle.roll;
        last_pitch = e.angle.pitch;

        int16_t dx = dr * K_ROLL;
        int16_t dy = dp * K_PITCH;
        dx = (dx>255?255: dx<-255?-255: dx);
        dy = (dy>255?255: dy<-255?-255: dy);

        event_t ev;
        ev.axis = 0; ev.val = dx; xQueueSend(xQueuePos, &ev, 0);
        ev.axis = 1; ev.val = dy; xQueueSend(xQueuePos, &ev, 0);
        if (abs(acceleration[0]) > CLICK_THRESHOLD) {
            if (!clicked) {
                ev.axis = 2; ev.val = 1;
                xQueueSend(xQueuePos, &ev, 0);
                clicked = true;
            }
        } else {
            clicked = false;
        }
        //printf("Acc. X = %d, Y = %d, Z = %d\n", acceleration[0], acceleration[1], acceleration[2]);
        //printf("Gyro. X = %d, Y = %d, Z = %d\n", gyro[0], gyro[1], gyro[2]);
        //printf("Temp. = %f\n", (temp / 340.0) + 36.53);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

int main() {
    stdio_init_all();

    xTaskCreate(mpu6050_task, "mpu6050_Task 1", 8192, NULL, 1, NULL);

    vTaskStartScheduler();

    while (true)
        ;
}
