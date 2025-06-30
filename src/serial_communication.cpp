#include "serial_communication.h"
#include <Arduino.h> // Để sử dụng Serial và các hàm Arduino khác

// Định nghĩa các hàm đã khai báo trong .h

void print_csv_header()
{
  Serial.println("Timestamp_ms,"
                 "Raw_Accel_X,Raw_Accel_Y,"

                 "Unbiased_Filt_Accel_X,Unbiased_Filt_Accel_Y,"

                 "Raw_Omega_Z_degs,Unbiased_Omega_Z_rads,"

                 "IMU_Yaw_deg," // Sắp xếp lại Roll, Pitch, Yaw

                 "GPS_Lon,GPS_Lat,GPS_GroundSpeed_kmh,GPS_Satellites,GPS_HDOP,"

                 "GPS_Local_Y_m,GPS_Local_X_m," // Thống nhất đơn vị
                 "EKF_lon,EKF_lat,"
                 "EKF_ap_Y_m,EKF_ap_X_m,"                                  // alpha_p (vị trí loci)
                 "EKF_av_X_mps,EKF_av_Y_mps,"                              // alpha_v (vận tốc loci)
                 "EKF_ba_X_mps2,EKF_ba_Y_mps2,"                            // Bias gia tốc kế
                 "EKF_bg_Z_rads,"                                          // Bias con quay hồi chuyển Z
                 "EKF_Yaw_deg,"                                            // Yaw ước tính từ EKF
                 "P_ap_X,P_ap_Y,P_av_X,P_av_Y,P_Yaw,P_ba_X,P_ba_Y,P_bg_Z," // P matrix diagonals
                 "beta_p_from_gps_actual_0,beta_p_from_gps_actual_1,"      // R matrix diagonals (2 phép đo)
                 "Q_av,Q_Yaw,Q_ba,Q_bg,"                                   // Q matrix diagonals (các nguồn nhiễu chính)
                 "z_x,z_y,"                                                // Innovation (đổi tên cho rõ ràng)
                 "ZUPT_Actived,EKF_Status,Delta_T_s");
}
void print_csv_header_2()
{
  Serial.println("Timestamp_ms,"
                 "Raw_Accel_X,Raw_Accel_Y,"
                 "Raw_Omega_Z_degs,"
                 "IMU_Yaw_deg," // Sắp xếp lại Roll, Pitch, Yaw
                 "GPS_Lon,GPS_Lat,GPS_GroundSpeed_kmh,"
                 "GPS_Local_X_m,GPS_Local_Y_m," // Thống nhất đơn vị
                 "EKF_lon,EKF_lat,"
                 "EKF_ap_X_m,EKF_ap_Y_m,"                // alpha_p (vị trí loci)
                 "EKF_av_X_mps,EKF_av_Y_mps,"            // alpha_v (vận tốc loci)
                 "EKF_ba_X_mps2,EKF_ba_Y_mps2,"          // Bias gia tốc kế
                 "EKF_bg_Z_rads,"                        // Bias con quay hồi chuyển Z
                 "EKF_Yaw_deg,ZUPT_Actived,EKF_Status"); // Yaw ước tính từ EKF
}

void print_csv_data(bool zupt_activated_flag, double current_delta_t_us)
{
  // Cột 1: Timestamp_ms
  // Serial.print(millis());
  Serial.print(current_data_index);
  Serial.print(",");

  // Cột 2-4: Gia tốc thô (Raw Accelerometer - m/s^2)
  Serial.print(g_accelX, 3); // Ax
  Serial.print(",");
  Serial.print(g_ax_body, 3); // Ay
  Serial.print(",");

  // Cột 5-6: Gia tốc đã xử lý (Unbiased & Filtered Accel - m/s^2)
  Serial.print(g_ax_body, 4); // Unbiased & Filtered Ax
  Serial.print(",");
  Serial.print(g_ay_body, 4); // Unbiased & Filtered Ay
  Serial.print(",");

  // Cột 7-9: Tốc độ góc thô (Raw Gyro - deg/s)
  Serial.print(g_angularVelZ, 4); // Raw Wz
  Serial.print(",");
  Serial.print(g_omega_body, 4); // Unbiased & Filtered Wz (rad/s) - in rad để phù hợp với EKF
  Serial.print(",");

  // Cột 10-12: Góc IMU (Roll, Pitch, Yaw - deg) - từ cảm biến
  Serial.print(degrees(g_yaw), 4); // Yaw (degrees)
  Serial.print(",");

  // Cột 13-17: Dữ liệu GPS thô và chất lượng
  Serial.print(g_longitude, 7); // Longitude (degrees)
  Serial.print(",");
  Serial.print(g_latitude, 7); // Latitude (degrees)
  Serial.print(",");
  Serial.print(g_groundSpeed, 2); // Ground Speed (km/h)
  Serial.print(",");
  Serial.print(g_svnum); // Number of Satellites
  Serial.print(",");
  Serial.print(g_hdop, 1); // HDOP
  Serial.print(",");

  // Cột 18-19: Tọa độ GPS đã chuyển đổi sang hệ tọa độ cục bộ (m)
  // Đây là gps_x, gps_y đã tính trong ExtendedKalmanFilter
  Serial.print(gps_y, 7); // GPS Y local
  Serial.print(",");
  Serial.print(gps_x, 7); // GPS X local
  Serial.print(",");

  Serial.print(ekf_EKF_Lon, 7); // GPS Y local
  Serial.print(",");
  Serial.print(ekf_EKF_Lat, 7); // GPS X local
  Serial.print(",");

  // Cột 20-30: Trạng thái EKF ước tính (11 trạng thái)
  Serial.print(ekf_ap_x, 4); // alpha_p_x (m)
  Serial.print(",");
  Serial.print(ekf_ap_y, 4); // alpha_p_y (m)
  Serial.print(",");
  Serial.print(ekf_av_x, 3); // alpha_v_x (m/s)
  Serial.print(",");
  Serial.print(ekf_av_y, 3); // alpha_v_y (m/s)
  Serial.print(",");

  Serial.print(ekf_ba_x, 4); // b_ax (m/s^2)
  Serial.print(",");
  Serial.print(ekf_ba_y, 4); // b_ay (m/s^2)
  Serial.print(",");
  Serial.print(ekf_bg_z, 4); // b_gz (rad/s)
  Serial.print(",");

  // Cột 31: Góc Yaw ước tính từ EKF (độ)
  Serial.print(degrees(ekf_yaw_rad), 2); // Yaw (degrees)
  Serial.print(",");

  // Cột 32-35: Ma trận hiệp phương sai P (đường chéo chính cho các trạng thái chính)
  Serial.print(*EEKF_MAT_EL(P, 0, 0), 3); // P(ap_x, ap_x)
  Serial.print(",");
  Serial.print(*EEKF_MAT_EL(P, 1, 1), 3); // P(ap_y, ap_y)
  Serial.print(",");
  Serial.print(*EEKF_MAT_EL(P, 2, 2), 3); // P(av_x, av_x)
  Serial.print(",");
  Serial.print(*EEKF_MAT_EL(P, 3, 3), 3); // P(av_y, av_y)
  Serial.print(",");
  Serial.print(*EEKF_MAT_EL(P, 4, 4), 3); // P(yaw, yaw)
  Serial.print(",");
  Serial.print(*EEKF_MAT_EL(P, 5, 5), 3); // P(b_ax, b_ax)
  Serial.print(",");
  Serial.print(*EEKF_MAT_EL(P, 6, 6), 3); // P(b_ay, b_ay)
  Serial.print(",");
  Serial.print(*EEKF_MAT_EL(P, 7, 7), 3); // P(b_gz, b_gz)
  Serial.print(",");

  // Cột 36-37: Ma trận nhiễu đo lường R (đường chéo chính) - giờ là 2x2
  Serial.print(beta_p_from_gps_actual_0, 7); // R cho delta_pos_x
  Serial.print(",");
  Serial.print(beta_p_from_gps_actual_1, 7); // R cho delta_pos_y
  Serial.print(",");

  // Cột 38-41: Ma trận nhiễu quá trình Q (đường chéo chính cho các nguồn nhiễu chính)
  Serial.print(*EEKF_MAT_EL(Q, 2, 2), 6); // Q cho av_x (từ nhiễu gia tốc)
  Serial.print(",");
  Serial.print(*EEKF_MAT_EL(Q, 4, 4), 2); // Q cho yaw (từ nhiễu gyro)
  Serial.print(",");
  Serial.print(*EEKF_MAT_EL(Q, 5, 5), 2); // Q cho b_ax (từ bias walk gia tốc)
  Serial.print(",");
  Serial.print(*EEKF_MAT_EL(Q, 7, 7), 62); // Q cho b_gz (từ bias walk gyro)
  Serial.print(",");
  Serial.print(*EEKF_MAT_EL(z, 0, 0), 6);
  Serial.print(",");
  Serial.print(*EEKF_MAT_EL(z, 1, 0), 6);
  Serial.print(",");
  // Cột cuối: Cờ ZUPT, Trạng thái EKF, Delta_T
  Serial.print(zupt_activated_flag ? "1" : "0"); // ZUPT active
  Serial.print(",");
  Serial.print(g_ekf_operation_status); // EKF status
  Serial.print(",");
  Serial.print(current_delta_t_us, 3); // Delta_T in seconds
  Serial.println();                    // Kết thúc dòng
}
void print_csv_data_2()
{
  // Cột 1: Timestamp_ms
  Serial.print(current_data_index);
  Serial.print(",");

  // Cột 10-12: Góc IMU (Roll, Pitch, Yaw - deg) - từ cảm biến
  Serial.print(g_accelX, 4); // Yaw (degrees)
  Serial.print(",");
  Serial.print(g_ax_body, 4); // Yaw (degrees)
  Serial.print(",");
  Serial.print(g_omega_body, 4); // Yaw (degrees)
  Serial.print(",");
  Serial.print(g_yaw, 4); // Yaw (degrees)
  Serial.print(",");

  // Cột 13-17: Dữ liệu GPS thô và chất lượng
  Serial.print(g_longitude, 7); // Longitude (degrees)
  Serial.print(",");
  Serial.print(g_latitude, 7); // Latitude (degrees)
  Serial.print(",");
  Serial.print(g_groundSpeed_filtered, 2); // Latitude (degrees)
  Serial.print(",");

  // Cột 18-19: Tọa độ GPS đã chuyển đổi sang hệ tọa độ cục bộ (m)
  // Đây là gps_x, gps_y đã tính trong ExtendedKalmanFilter
  Serial.print(gps_x, 7); // GPS X local
  Serial.print(",");
  Serial.print(gps_y, 7); // GPS Y local
  Serial.print(",");

  Serial.print(ekf_EKF_Lon, 7); // GPS Y local
  Serial.print(",");
  Serial.print(ekf_EKF_Lat, 7); // GPS X local
  Serial.print(",");

  // Cột 20-30: Trạng thái EKF ước tính (11 trạng thái)
  Serial.print(ekf_ap_x, 2); // alpha_p_x (m)
  Serial.print(",");
  Serial.print(ekf_ap_y, 2); // alpha_p_y (m)
  Serial.print(",");
  Serial.print(ekf_av_x, 2); // alpha_v_x (m/s)
  Serial.print(",");
  Serial.print(ekf_av_y, 2); // alpha_v_y (m/s)
  Serial.print(",");

  Serial.print(ekf_ba_x, 4); // b_ax (m/s^2)
  Serial.print(",");
  Serial.print(ekf_ba_y, 4); // b_ay (m/s^2)
  Serial.print(",");
  Serial.print(ekf_bg_z, 4); // b_gz (rad/s)
  Serial.print(",");

  // Cột 31: Góc Yaw ước tính từ EKF (độ)
  Serial.print(degrees(ekf_yaw_rad), 2); // Yaw (degrees)
  Serial.print(",");
  Serial.print(zupt_was_activated);
  Serial.print(",");
  Serial.print(g_ekf_operation_status);
  Serial.println();
}

void handleSerialInput()
{
  String inputString = Serial.readStringUntil('\n');
  inputString.trim();
  if (inputString.length() == 0)
    return;

  Serial.print("Da nhan: [");
  Serial.print(inputString);
  Serial.println("]");

  double tempValues[6]; // Mảng để lưu 5 giá trị float
  int valueCount = 0;
  int currentIndex = 0;
  int nextCommaIndex = 0;

  while (valueCount < 6 && currentIndex < inputString.length())
  {
    nextCommaIndex = inputString.indexOf(',', currentIndex);
    String token;
    if (nextCommaIndex == -1) // Không tìm thấy dấu phẩy nữa, lấy phần còn lại
    {
      token = inputString.substring(currentIndex);
      currentIndex = inputString.length(); // Kết thúc vòng lặp
    }
    else
    {
      token = inputString.substring(currentIndex, nextCommaIndex);
      currentIndex = nextCommaIndex + 1; // Di chuyển đến sau dấu phẩy
    }
    token.trim();
    if (token.length() > 0)
    {
      tempValues[valueCount] = token.toFloat();
      valueCount++;
    }
  }

  if (valueCount == 6) // 0.3,0.01,0.0001,0.00001,0.01,0
  {
    g_sigma_accel_noise = tempValues[0];
    g_sigma_gyro_noise = tempValues[1];
    g_sigma_accel_bias_walk = tempValues[2];
    g_sigma_gyro_bias_walk = tempValues[3];
    g_std_dev_gps_pos = tempValues[4]; // Tham số thứ 5
    float ef = tempValues[5];          // Tham số thứ 5

    Serial.println("Cap nhat cac tham so EKF thanh cong!");
    // Serial.printf("sigma_accel: %.4f, sigma_omega: %.4f, std_gps_pos: %.3f, std_gps_vel: %.3f\n",
    //               g_sigma_accel_process, g_sigma_omega_process, g_std_dev_gps_pos, g_std_dev_gps_vel);
    update_Q_matrix(dT); // Cập nhật Q với các sigma mới và dT hiện tại
    update_R_matrix();   // Cập nhật R với các sigma mới và các yếu tố chất lượng GPS hiện tại
  }
  else
  {
    Serial.print("Loi: Dinh dang hoac so luong tham so khong dung. Mong doi 5 gia tri float.\n");
  }
}
// void printDebug1()
// {
//   Serial.print("gps_x:");
//   Serial.print(gps_x);
//   Serial.print(",");
//   Serial.print("gps_y:");
//   Serial.print(gps_y);
//   Serial.print(",");
//   Serial.print("gps_vx:");
//   Serial.print(gps_vx);
//   Serial.print(",");
//   Serial.print("gps_vy:");
//   Serial.print(gps_vy);
//   Serial.print(",");
//   Serial.print("ekf_x_pos:");
//   Serial.print(ekf_x_pos);
//   Serial.print(",");
//   Serial.print("ekf_y_pos:");
//   Serial.print(ekf_y_pos);
//   Serial.print(",");
//   Serial.print("ekf_vx_vel:");
//   Serial.print(ekf_vx_vel);
//   Serial.print(",");
//   Serial.print("ekf_vy_vel:");
//   Serial.print(ekf_vy_vel);
//   Serial.print(",");
//   Serial.print("ekf_theta_ang:");
//   Serial.print(ekf_theta_ang);
//   Serial.print(",");
//   Serial.print("g_accelX:");
//   Serial.print(g_accelX);
//   Serial.print(",");
//   Serial.print("g_ax_body:");
//   Serial.print(g_ax_body);
//   Serial.print(",");
//   Serial.print("g_angularVelZ:");
//   Serial.print(g_angularVelZ);
//   Serial.print(",");
//   Serial.print("g_omega_body:");
//   Serial.print(g_omega_body);
//   Serial.print(",");
//   Serial.print("g_yaw:");
//   Serial.print(g_yaw);
//   Serial.print(",");
//   Serial.print("g_latitude:");
//   Serial.print(g_latitude);
//   Serial.print(",");
//   Serial.print("g_longitude:");
//   Serial.print(g_longitude);
//   Serial.print(",");
//   Serial.print("g_groundSpeed:");
//   Serial.print(g_groundSpeed);
//   Serial.print(",");
//   Serial.print("g_svnum:");
//   Serial.print(g_svnum);
//   Serial.print(",");
//   Serial.print("g_hdop:");
//   Serial.print(g_hdop);
//   Serial.print(",");
//   Serial.print("g_ekf_operation_status:");
//   Serial.println(g_ekf_operation_status);
// }

// Hàm printDebug2 không còn được sử dụng trong main.cpp
// void printDebug2()
// {
//   // ... (code gốc) ...
// }

// Hàm printDebug1 cũng không còn được sử dụng trong main.cpp
// void printDebug1() { /* ... */ }
void printDebug2()
{
  Serial.print("g_accelX:");
  Serial.print(g_accelX);
  Serial.print(",");
  Serial.print("g_ax_body:");
  Serial.print(g_ax_body);
  Serial.print(",");
  Serial.print("g_yaw:");
  Serial.print(g_yaw);
  Serial.print(",");
  Serial.print("g_latitude:");
  Serial.print(g_latitude);
  Serial.print(",");
  Serial.print("g_longitude:");
  Serial.print(g_longitude);
  Serial.print(",");
  Serial.print("g_groundSpeed:");
  Serial.print(g_groundSpeed);
  Serial.print(",");
  Serial.print("g_svnum:");
  Serial.print(g_svnum);
  Serial.print(",");
  Serial.print("g_hdop:");
  Serial.print(g_hdop);
  Serial.print(",");
  Serial.print("g_ekf_operation_status:");
  Serial.println(g_ekf_operation_status);
}
