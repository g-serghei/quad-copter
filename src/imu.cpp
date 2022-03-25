#include <stdarg.h>
#include <stdio.h>
#include "imu.h"
#include "log.h"

namespace Imu
{
    static float GYRO_PART = 0.998;
    static float ACC_PART = 1.0 - GYRO_PART;

    bfs::Mpu9250 sensor(&Wire, 0x68);

    bool dataAvailable = false;
    float lastReadMicros;

    MeridialFilterType accelFilterData;

    AxisType gyroAngles = {0.0, 0.0, 0.0};
    AxisType radAngles = {0.0, 0.0, 0.0};
    AxisType degAngles = {0.0, 0.0, 0.0};

    ImuType rawData = {
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0}};

    ImuType data = {
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0}};

    ImuType dataOffset = {
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0}};

    void init()
    {
        Wire.begin();
        Wire.setClock(400000);

        delay(3000);

        if (!sensor.Begin())
        {
            Log::error("Error connecting to Mpu9250");
        }

        if (!sensor.ConfigAccelRange(bfs::Mpu9250::ACCEL_RANGE_16G))
        {
            Log::error("Error ConfigAccelRange");
        }

        if (!sensor.ConfigGyroRange(bfs::Mpu9250::GYRO_RANGE_2000DPS))
        {
            Log::error("Error ConfibfsgGyroRange");
        }

        // if (!sensor.ConfigSrd(19))
        // {
        //     Log::error("Error configured SRD");
        // }

        // if (!sensor.EnableDrdyInt())
        // {
        //     Log::error("Error EnableDrdyInt");
        // }

        // pinMode(19, INPUT);
        // attachInterrupt(19, recordRawData, RISING);

        lastReadMicros = micros();

        byte mSize = 11;

        accelFilterData.x = median_filter_new(mSize, 0.0);
        accelFilterData.y = median_filter_new(mSize, 0.0);
        accelFilterData.z = median_filter_new(mSize, 0.0);

        calibrate();
    }

    void calibrate()
    {
        delay(3000);
        serialPrintlnf("Calibrating Accel / Gyro...");

        int times = 5000;

        AxisType accelSum = {0.0, 0.0, 0.0};
        AxisType gyroSum = {0.0, 0.0, 0.0};

        for (int i = 0; i < times; i++)
        {
            updateData();

            accelSum.x += rawData.accel.x;
            accelSum.y += rawData.accel.y;
            accelSum.z += rawData.accel.z;

            gyroSum.x += rawData.gyro.x;
            gyroSum.y += rawData.gyro.y;
            gyroSum.z += rawData.gyro.z;

            delay(1);
        }

        dataOffset.accel.x = accelSum.x / times;
        dataOffset.accel.y = accelSum.y / times;
        dataOffset.accel.z = accelSum.z / times;

        dataOffset.gyro.x = gyroSum.x / times;
        dataOffset.gyro.y = gyroSum.y / times;
        dataOffset.gyro.z = gyroSum.z / times;

        delay(1000);
    }

    void updateData()
    {
        if (!sensor.Read())
        {
            return;
        }

        // Raw data
        rawData.accel.x = sensor.accel_x_mps2();
        rawData.accel.y = sensor.accel_y_mps2();
        rawData.accel.z = sensor.accel_z_mps2();

        rawData.gyro.x = sensor.gyro_x_radps();
        rawData.gyro.y = sensor.gyro_y_radps();
        rawData.gyro.z = sensor.gyro_z_radps();

        // Data with offset
        data.accel.x = rawData.accel.x - dataOffset.accel.x;
        data.accel.y = rawData.accel.y - dataOffset.accel.y;
        data.accel.z = -1 * ((rawData.accel.z - dataOffset.accel.z) - 9.80665f);

        data.gyro.x = rawData.gyro.x - dataOffset.gyro.x;
        data.gyro.y = rawData.gyro.y - dataOffset.gyro.y;
        data.gyro.z = rawData.gyro.z - dataOffset.gyro.z;

        // Apply Median Filter on Accel data
        median_filter_in(accelFilterData.x, data.accel.x);
        median_filter_in(accelFilterData.y, data.accel.y);
        median_filter_in(accelFilterData.z, data.accel.z);

        data.accel.x = (float)(median_filter_out(accelFilterData.x));
        data.accel.y = (float)(median_filter_out(accelFilterData.y));
        data.accel.z = (float)(median_filter_out(accelFilterData.z));

        dataAvailable = true;
    }

    void process()
    {
        Imu::updateData();

        if (!dataAvailable)
        {
            return;
        }

        dataAvailable = false;

        AxisType accelAngles = {0.0, 0.0, 0.0};

        accelAngles.x = atan2(-1 * data.accel.y, data.accel.z);
        accelAngles.y = atan2(-1 * data.accel.x, sqrt(data.accel.y * data.accel.y + data.accel.z * data.accel.z));

        float dt = (float)(micros() - lastReadMicros) / 1000000;

        radAngles.x = GYRO_PART * (radAngles.x + (data.gyro.x * dt)) + (ACC_PART * accelAngles.x);
        radAngles.y = GYRO_PART * (radAngles.y + (data.gyro.y * dt)) + (ACC_PART * accelAngles.y);

        gyroAngles.x = gyroAngles.x + (data.gyro.x * dt);
        gyroAngles.y = gyroAngles.y + (data.gyro.y * dt);
        gyroAngles.z = gyroAngles.z + (data.gyro.z * dt);

        degAngles = {(float)(radAngles.x * RAD_TO_DEG), (float)(radAngles.y * RAD_TO_DEG), (float)(gyroAngles.z * RAD_TO_DEG)};

        // Serial.print("GyroX:");
        // Serial.print(gyroAngles.x * RAD_TO_DEG);
        // Serial.print(",");
        // Serial.print("AccelX:");
        // Serial.print(accelAngles.x * RAD_TO_DEG);
        // Serial.print(",");
        // Serial.print("CompX:");
        // Serial.println(radAngles.x * RAD_TO_DEG);

        lastReadMicros = micros();
    }

    AxisType getDegAngles()
    {
        return degAngles;
    }

    void printAxis(AxisType axis)
    {
        serialPrintlnf("x: %f, y:%f, z:%f", axis.x, axis.y, axis.z);
    }

    void serialPrintf(const char *format, ...)
    {
        char buffer[256];

        va_list args;
        va_start(args, format);
        vsprintf(buffer, format, args);

        Serial.write(buffer, strlen(buffer));

        va_end(args);
    }

    void serialPrintlnf(const char *format, ...)
    {
        char buffer[256];

        va_list args;
        va_start(args, format);
        vsprintf(buffer, format, args);

        char newline_buffer[256];
        sprintf(newline_buffer, "%s\r\n", buffer);

        Serial.write(newline_buffer, strlen(newline_buffer));

        va_end(args);
    }
}