#ifndef SENSOR_PROCESSING_H
#define SENSOR_PROCESSING_H

#include "globals.h" // Để truy cập các biến toàn cục và hằng số

// Khai báo các hàm
void calibrate_sensors();
float calculate_std_dev_optimized(double sum, double sum_sq, int count);
bool is_stationary(bool p_is_gps_vel_valid_in);
// Bạn có thể thêm các hàm xử lý cảm biến khác ở đây nếu cần

#endif // SENSOR_PROCESSING_H
