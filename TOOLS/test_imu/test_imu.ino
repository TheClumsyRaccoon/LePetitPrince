#include <Wire.h>
#include "ICM45686.h"

#define PIN_SDA 8
#define PIN_SCL 9

ICM456xx IMU(Wire, 0);   /* 0 = adresse 0x68, 1 = 0x69 */

static const float FSR_ACC_G   = 4.0f;
static const float FSR_GYR_DPS = 500.0f;
static const float RAD2DEG     = 57.2957795f;

void setup() {
    Serial.begin(115200);
    delay(1000);
    Wire.begin(PIN_SDA, PIN_SCL);

    int code = IMU.begin();
    if (code != 0) {
        Serial.print("Erreur begin : ");
        Serial.println(code);
        while (true) delay(1000);
    }
    IMU.startAccel(200, 4);     /* 200 Hz, +-4 g */
    IMU.startGyro(200, 500);    /* 200 Hz, +-500 dps */
    delay(100);
}

void loop() {
    inv_imu_sensor_data_t d;
    if (IMU.getDataFromRegisters(d) != 0) {
        Serial.println("Erreur lecture");
        delay(500);
        return;
    }
    float ax = d.accel_data[0] * FSR_ACC_G / 32768.0f;
    float ay = d.accel_data[1] * FSR_ACC_G / 32768.0f;
    float az = d.accel_data[2] * FSR_ACC_G / 32768.0f;
    float gx = d.gyro_data[0] * FSR_GYR_DPS / 32768.0f;
    float gy = d.gyro_data[1] * FSR_GYR_DPS / 32768.0f;
    float gz = d.gyro_data[2] * FSR_GYR_DPS / 32768.0f;

    float norme = sqrtf(ax * ax + ay * ay + az * az);
    float roll  = atan2f(ay, az) * RAD2DEG;
    float pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD2DEG;

    Serial.printf("acc[g] %.2f %.2f %.2f | norme %.2f | gyro[dps] %.1f %.1f %.1f | roll %.0f pitch %.0f\n",
                  ax, ay, az, norme, gx, gy, gz, roll, pitch);
    delay(200);
}