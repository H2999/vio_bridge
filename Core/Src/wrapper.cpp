#include <string.h>

#include "mpu6050.h"
#include "bsp_dwt.h"
#include "main.h"

// 全局对象（C++ 对象）
static MPU6050& mpu = MPU6050::getinstance();

// C 接口函数：初始化 IMU
extern "C" HAL_StatusTypeDef mpu6050_init_wrapper(void)
{
    return mpu.MPU6050_Init();
}

// C 接口函数：获取 IMU 数据（返回指针，避免引用）
extern "C" void mpu6050_get_data_wrapper(float* acc, float* gyro)
{
    const MPU6050::mpu6050_t& data = mpu.get_data();
    memcpy(acc, data.accel_final, sizeof(float) * 3);
    memcpy(gyro, data.gyro_final, sizeof(float) * 3);
}

// C 接口函数：获取 DWT 时间
extern "C" uint64_t dwt_get_timeline_us_wrapper(void)
{
    return DWT_GetTimeline_us();
}