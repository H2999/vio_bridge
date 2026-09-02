#ifndef __WRAPPER_H
#define __WRAPPER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

    int mpu6050_init_wrapper(void);
    void mpu6050_get_data_wrapper(float* acc, float* gyro);
    uint64_t dwt_get_timeline_us_wrapper(void);

#ifdef __cplusplus
}
#endif

#endif