#pragma once

#include <stdint.h>

/* Per-IMU gyro statistics computed on the CM4 from every sensor_gyro sample
 * and published once a second per instance as debug_array "cm4gyro"
 * (id = slot). Layout of data[]:
 *   0 device_id low 16 bits   1 device_id high 16 bits
 *   2 samples/s               3 largest sample gap (us)
 *   4..6  mean x y z (rad/s)
 *   7..9  rms of the mean-removed signal x y z (vibration, rad/s)
 *  10..12 peak |x| |y| |z| of the mean-removed signal (rad/s)
 *  13..15 dominant frequency estimate x y z (Hz, zero crossings / 2)
 */

#define GYRO_ANALYSER_SLOTS 3

void gyro_analyser_init(int debug_array_handle);
void gyro_analyser_sample(const void *sensor_gyro, uint16_t size);
void gyro_analyser_poll(uint32_t now_us);
