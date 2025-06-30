#include <Arduino.h> // Bao gồm thư viện Arduino
#include <ModbusMaster.h>

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "globals.h"
#include "ekf_handler.h"

#include <WTGAHRS3_485.h>

void setup()
{
  delay(1000);             // Chờ một chút để hệ thống ổn định
  setCpuFrequencyMhz(240); // Set CPU frequency to 240 MHz
  delay(1000);             // Wait for the system to stabilize
  Serial.begin(115200);
  delay(500); // Wait for Serial to initialize
  RS485.begin(115200, SERIAL_8N1, RXD2, TXD2);
  delay(500); // Wait for RS485 to initialize
  node.begin(SENSOR_SLAVE_ID, RS485);
  delay(500); // Wait for ModbusMaster to initialize)

  // Serial.println(sensor.bandwidthToString(sensor.getBandwidth()));
  // delay(500); // Wait for the sensor to respond
  // sensor.setBaudRate(BAUD_115200);
  // delay(500);
  sensor.setDataResponseDelay(1000); // Đặt độ trễ phản hồi dữ liệu (us)
  delay(500);                        // Chờ cảm biến áp dụng cấu hình
  // sensor.setBandwidth(BW_188_HZ);
  // delay(500);                                             // Chờ cảm biến hoàn thành hiệu chỉnh tự động
  // sensor.setCalibrationCommand(SET_HEADING_ANGLE_TO_ZERO); // Hiệu chỉnh yaw về 0
  // delay(500);
  // sensor.setCalibrationCommand(AUTO_ADDER_CALIBRATION);
  // delay(20000);
  // sensor.setCalibrationCommand(SET_HEADING_ANGLE_TO_ZERO);
  // delay(10000);
  // sensor.saveConfiguration();
  // delay(500); // Chờ cảm biến hoàn thành hiệu chỉnh yaw
  // sensor.rebootDevice();
  // delay(5000); // Chờ cảm biến hoàn thành khởi động lại
  Serial.println("Cảm biến đã khởi động lại và sẵn sàng.");

  // memset(C_b_n0_mat_global, 0, sizeof(C_b_n0_mat_global));
  // C_b_n0_mat_global[0] = (eekf_value)1.0;
  // C_b_n0_mat_global[4] = (eekf_value)1.0;
  // C_b_n0_mat_global[8] = (eekf_value)1.0;
}

unsigned long last_print_time_us = 0;           // Thời điểm cuối cùng print_csv_data được gọi
const unsigned long PRINT_INTERVAL_US = 200000; // Tần suất in dữ liệu CSV (200ms = 5Hz)

const unsigned long IMU_READ_INTERVAL_US = 5319;   // Khoảng 188Hz
// const unsigned long GPS_READ_INTERVAL_US = 200000; // 5Hz
const unsigned long GPS_READ_INTERVAL_US = 1000000; // 1Hz
const unsigned long SERIAL_INTERVAL_US = 20000; // 20ms = 50Hz

unsigned long last_imu_read_time_us = 0;
unsigned long last_gps_read_time_us = 0;
unsigned long last_serial_time_us = 0;

unsigned long delta_t_us = 0;
unsigned long delta_t_gps_us = 0;
unsigned long delta_t_serial_us = 0;

unsigned long current_loop_time_us = 0;
int gps_count = 1; // Biến đếm số lần đọc GPS
int imu_count = 1; // Biến đếm số lần đọc GPS
void loop()
{
  if (origin_set)
  {
    current_loop_time_us = micros() - after_init; // Tính thời gian đã trôi qua kể từ khi khởi động
    delta_t_us = current_loop_time_us - last_imu_read_time_us;
    delta_t_gps_us = current_loop_time_us - last_gps_read_time_us;
    delta_t_serial_us = current_loop_time_us - last_serial_time_us;
  }

  if (is_stationary(true) && origin_set) // Hàm is_stationary cần được triển khai và tin cậy
  {
    zupt_was_activated = true;

    // Đặt vận tốc về 0
    *EEKF_MAT_EL(*ctx.x, 2, 0) = 0.0; // alpha_v_x
    *EEKF_MAT_EL(*ctx.x, 3, 0) = 0.0; // alpha_v_y

    // Giảm hiệp phương sai của vận tốc
    *EEKF_MAT_EL(*ctx.P, 2, 2) *= 0.1;                       // Giảm 10 lần
    *EEKF_MAT_EL(*ctx.P, 3, 3) = *EEKF_MAT_EL(*ctx.P, 2, 2); // Đồng bộ P_y = P_x

    // Đặt các hiệp phương sai chéo liên quan đến vận tốc về 0
    for (int i = 0; i < NUM_STATES; i++)
    {
      if (i != 2)
      {
        *EEKF_MAT_EL(*ctx.P, i, 2) = 0.0; // Cột 2
        *EEKF_MAT_EL(*ctx.P, 2, i) = 0.0; // Hàng 2
      }
      if (i != 3)
      {
        *EEKF_MAT_EL(*ctx.P, i, 3) = 0.0; // Cột 3
        *EEKF_MAT_EL(*ctx.P, 3, i) = 0.0; // Hàng 3
      }
    }

    // Cập nhật trạng thái EKF
    //g_ekf_operation_status = EKF_STATUS_PREDICT_UPDATE_ZUPT;
  }
  else
  {
    zupt_was_activated = false;
  } // Reset cờ ZUPT

  // --- Đọc dữ liệu IMU và Attitude ở ~188Hz ---
  if ((delta_t_us >= IMU_READ_INTERVAL_US) && origin_set)
  {
    last_imu_read_time_us = current_loop_time_us;

    // Serial.println("Đọc dữ liệu IMU và Attitude... ");

    if (delta_t_us > 2 * IMU_READ_INTERVAL_US)
      delta_t_us = 2 * IMU_READ_INTERVAL_US; // Giới hạn delta_t_us tối đa

    // SynchronizedImuAttitudeData imu_att_data = sensor.getSynchronizedImuAttitudeData();
    imu_att_data.isDataValid = true;
    if (imu_att_data.isDataValid)
    {
      isIMUDataValid = true; // Đặt cờ dữ liệu IMU hợp lệ
      g_accelX = Raw_Accel_X[current_data_index];
      g_accelY = Raw_Accel_Y[current_data_index];
      g_angularVelZ = radians(Raw_Omega_Z_degs[current_data_index]) * -1.0f; // Đơn vị: deg/s
      g_yaw = IMU_Yaw_deg[current_data_index];                            // Đơn vị: deg
      
      g_yaw_body = radians(g_yaw);
      if (current_data_index >= NUM_SAMPLE_RECORDS) // Kiểm tra nếu đã đọc hết dữ liệu
      {
        // current_data_index = 0; // Reset index nếu vượt quá số điểm dữ liệu
        // num_gps_points_collected = 0;
        Serial.println();
        Serial.println();
        Serial.println();
        delay(500000);
        return;
      }

      // Áp dụng LPF cho gia tốc thô

      g_ax_filtered = LPF_ALPHA * g_accelX + (1.0 - LPF_ALPHA) * g_ax_filtered;
      g_ay_filtered = LPF_ALPHA * g_accelY + (1.0 - LPF_ALPHA) * g_ay_filtered;

      // // Sử dụng gia tốc đã lọc và đã hiệu chỉnh bias cho EKF
      // g_ax_body = g_ax_filtered - g_ax_bias;
      // g_ay_body = g_ay_filtered - g_ay_bias;
      g_ax_body = g_ax_filtered; // Gia tốc X đã hiệu chỉnh bias
      g_ay_body = g_ax_filtered; // Gia tốc X đã hiệu chỉnh bias
      // Áp dụng ngưỡng để đưa g_ax_body về 0 nếu nó rất nhỏ
      // if (fabs(g_ax_body) < G_A_BODY_ZERO_THRESHOLD)
      // {
      //   g_ax_body = 0.0f;
      // }
      // if (fabs(g_ay_body) < G_A_BODY_ZERO_THRESHOLD)
      // {
      //   g_ay_body = 0.0f;
      // }

      // float omega_rads_unbiased = g_angularVelZ;
      // // Áp dụng LPF cho vận tốc góc đã hiệu chỉnh bias (rad/s)
      // g_omega_filtered_rads = (LPF_ALPHA * omega_rads_unbiased) + (1.0 - LPF_ALPHA) * g_omega_filtered_rads;
      // // Sử dụng vận tốc góc đã hiệu chỉnh bias (rad/s) cho EKF
      // g_omega_body = g_omega_filtered_rads - g_omega_bias;
      g_omega_body = g_angularVelZ;

      //  Đảm bảo yaw nằm trong khoảng [-PI, PI]
      g_yaw_body = fmod(g_yaw_body + M_PI, 2.0 * M_PI); // Chuẩn hóa về khoảng [-PI, PI]
      if (g_yaw_body < 0.0)
      {
        g_yaw_body += 2.0 * M_PI;
      }
      g_yaw_body -= M_PI;
    }
    else
    { // Xử lý lỗi đọc Gyro
      // Serial.println("Loi: Khong doc duoc du lieu IMU/Attitude dong bo.");
      isIMUDataValid = false; // Đặt cờ dữ liệu IMU không hợp lệ
    }
    /* Serial.print("g_ax_body:");
    Serial.print(g_ax_body, 2);
    Serial.print(",");
    Serial.print("g_ay_body:");
    Serial.print(g_ay_body, 2);
    Serial.print(",");
    Serial.print("g_omega_body:");
    Serial.println(g_omega_body, 2); */
    // Logic EKF và origin_set (chạy sau khi có dữ liệu IMU mới)
    // Sử dụng các cờ GPS từ lần đọc GPS gần nhất

    if (origin_set && isIMUDataValid)
    {
      dT = double(delta_t_us / 1000000.0);

      // Serial.printf("delta_t_us: %lu, dT: %.6f s\n", delta_t_us, dT);
      update_Q_matrix(dT);
      if (Serial.available() > 0)
      {
        handleSerialInput();
      }

      g_ekf_operation_status = EKF_STATUS_NONE; // Reset trạng thái ở đầu mỗi lần gọi

      // Cập nhật đầu vào của bộ lọc Kalman (u)
      *EEKF_MAT_EL(u, 0, 0) = g_ax_body;    // g_ax_body là gia tốc đã lọc và trừ bias
      *EEKF_MAT_EL(u, 1, 0) = g_ay_body;    // g_ay_body
      *EEKF_MAT_EL(u, 2, 0) = g_omega_body; // Vận tốc góc Z trừ bias ước tính
      // Serial.printf("current_data_index: %d,,ekf_ap_x: %.3f, ekf_ap_y: %.3f, ekf_av_x: %.3f, ekf_av_y: %.3f, ekf_ba_x: %.3f, ekf_ba_y: %.3f, ekf_bg_z: %.3f, ekf_yaw_rad: %.3f\n",
      //               current_data_index,ekf_ap_x, ekf_ap_y, ekf_av_x, ekf_av_y, ekf_ba_x, ekf_ba_y, ekf_bg_z, degrees(ekf_yaw_rad));

      eekf_predict(&ctx, &u, &Q);
      current_data_index++;
      // g_ekf_operation_status = EKF_STATUS_PREDICT_ONLY;

      ekf_ap_x = double(*EEKF_MAT_EL(*ctx.x, 0, 0));
      ekf_ap_y = double(*EEKF_MAT_EL(*ctx.x, 1, 0));
      ekf_av_x = double(*EEKF_MAT_EL(*ctx.x, 2, 0));
      ekf_av_y = double(*EEKF_MAT_EL(*ctx.x, 3, 0));
      ekf_yaw_rad = double(*EEKF_MAT_EL(*ctx.x, 4, 0));
      ekf_ba_x = double(*EEKF_MAT_EL(*ctx.x, 5, 0));
      ekf_ba_y = double(*EEKF_MAT_EL(*ctx.x, 6, 0));
      ekf_bg_z = double(*EEKF_MAT_EL(*ctx.x, 7, 0));
      ekf_EKF_Lat = lat_origin + (ekf_ap_y / METERS_PER_DEG_LAT);
      ekf_EKF_Lon = lon_origin + (ekf_ap_x / METERS_PER_DEG_LON);

      // print_csv_data(zupt_was_activated, delta_t_us);
      // if(imu_count% 10 == 0) // In dữ liệu mỗi 10 lần đọc IMU
      // {
      //   imu_count = 1; // Reset biến đếm
      //   print_csv_data_2();
      // }
      // else
      // {
      //   imu_count++;
      // }
      // print_csv_data_2();
    }
  }

  // --- Đọc dữ liệu GPS ở 5Hz ---
  if ((delta_t_gps_us >= GPS_READ_INTERVAL_US) && origin_set)
  {
    // delay(100000);
    last_gps_read_time_us = current_loop_time_us;
    // Serial.println("Đọc dữ liệu GPS... ");
    // delta_t_gps_us = 0;
    // gps_data_local = sensor.getSynchronizedGpsData();
    gps_data_local.isDataValid = true;
    if (gps_data_local.isDataValid)
    {
      isGPSDataValid = true;

      g_latitude = GPS_Lat[current_data_index];
      g_longitude = GPS_Lon[current_data_index];
      g_svnum = GPS_Satellites[current_data_index]; // Số lượng vệ tinh
      g_hdop = GPS_HDOP[current_data_index];        // HDOP

      // METERS_PER_DEG_LON = 111320.0 * cos(radians(lat_origin));
      gps_count++;
      
      // Áp dụng LPF cho groundSpeed
      g_groundSpeed_filtered_kmh_state = GPS_GroundSpeed_kmh[current_data_index];
      g_groundSpeed = g_groundSpeed_filtered_kmh_state * 1000.0f / 3600.0f; // Chuyển đổi từ km/h sang m/s

      g_groundSpeed_filtered = (0.2 * g_groundSpeed) + (1.0 - 0.2) * g_groundSpeed_filtered;

      gps_history[gps_history_idx].latitude = g_latitude;
      gps_history[gps_history_idx].longitude = g_longitude;
      gps_history[gps_history_idx].timestamp_ms = millis();
      // Serial.printf("current_data_index: %d\n", current_data_index);
      // Tính toán và lưu tọa độ cục bộ ngay lập tức
      geo_to_local_enu(g_latitude, g_longitude, lat_origin, lon_origin,
                       &gps_history[gps_history_idx].local_x,
                       &gps_history[gps_history_idx].local_y);
      gps_x = gps_history[gps_history_idx].local_x;
      gps_y = gps_history[gps_history_idx].local_y;
      // Serial.printf("gps_history_idx %d\n", gps_history_idx);
      // Serial.printf("gps_x: %.3f, gps_y: %.3f\n",
      //               gps_x, gps_y);
      gps_history_idx = (gps_history_idx + 1) % MAX_GPS_HISTORY_POINTS;

      if (num_gps_points_collected < MAX_GPS_HISTORY_POINTS)
      {
        num_gps_points_collected++;
      }
      if (gps_count > 200)
      {
        Serial.println();
        delay(100000);
      }
      // Serial.printf("num_gps_points_collected: %d\n", num_gps_points_collected);
      bool attempt_gps_update = (g_svnum >= GPS_QUALITY_MIN_SATS) &&
                                (g_hdop <= GPS_QUALITY_MAX_HDOP) &&
                                (g_hdop > 0.0f);
      if (attempt_gps_update) // CHỈ CHẠY KHỐI NÀY KHI CÓ DỮ LIỆU GPS MỚI HỢP LỆ
      {
        // Bước 1: Tính toán beta_p_from_gps_actual (vector quan sát PIF từ GPS)
        // (Sử dụng lịch sử GPS)
        eekf_value beta_p_from_gps_actual[2]; // 2D vector
        if (num_gps_points_collected >= 1)    // Đảm bảo có ít nhất 1 điểm GPS trong lịch sử
        {
          calculate_beta_p(num_gps_points_collected, beta_p_from_gps_actual);
        }
        else // Nếu chưa có điểm nào, không thể tính beta_p, không cập nhật
        {
          // Có thể in cảnh báo hoặc đặt cờ để bỏ qua cập nhật này
          // Để đảm bảo beta_p không có giá trị rác, ta đặt 0.0
          beta_p_from_gps_actual[0] = 0.0;
          beta_p_from_gps_actual[1] = 0.0;
          attempt_gps_update = false; // Không cập nhật nếu chưa có đủ lịch sử GPS
        }
        // Serial.printf("num_gps_points_collected: %d, beta_p_from_gps_actual: %.3f, %.3f\n",
        //               num_gps_points_collected, beta_p_from_gps_actual[0], beta_p_from_gps_actual[1]);

        beta_p_from_gps_actual_0 = beta_p_from_gps_actual[0];
        beta_p_from_gps_actual_1 = beta_p_from_gps_actual[1];
        // Serial.printf("num_gps_points_collected %d, beta_p_from_gps_actual: %.3f, %.3f\n", num_gps_points_collected, beta_p_from_gps_actual[0], beta_p_from_gps_actual[1]);
        //  Serial.println();
        if (attempt_gps_update) // Kiểm tra lại sau khi tính beta_p
        {
          // Bước 2: Lấy alpha_p hiện tại từ trạng thái bộ lọc
          eekf_value current_alpha_p_from_state[2] = {*EEKF_MAT_EL(*ctx.x, 0, 0), *EEKF_MAT_EL(*ctx.x, 1, 0)};

          // Bước 3: Tính toán C_b^n(0)_k (Initial Attitude Matrix) bằng OBA
          // Hàm này sẽ cập nhật C_b_n0_mat_global
          // if (gps_count > 2 || gps_count % 10 == 0)
          // {
          //   calculate_initial_attitude_OBA(current_alpha_p_from_state, beta_p_from_gps_actual, C_b_n0_mat_global);
          //   // Serial.println("C_b_n0_mat_global đã được cập nhật bằng OBA.");
          //   // Serial.printf("C_b_n0_mat_global: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]\n",
          //   //               C_b_n0_mat_global[0], C_b_n0_mat_global[1], C_b_n0_mat_global[2],
          //   //               C_b_n0_mat_global[3], C_b_n0_mat_global[4], C_b_n0_mat_global[5],
          //   //               C_b_n0_mat_global[6], C_b_n0_mat_global[7], C_b_n0_mat_global[8]);
          // }

          // C_b_n0_mat_global bây giờ đã được tính toán lại bằng OBA.

          // --- Tính toán vector đo lường Z (innovation) cho EKF ---
          // Z_k = beta_p,k (observed) - C_b^n(0)_k * alpha_p,k (predicted) (Equation 29)
          // predicted_beta_p_from_state = C_b_n0_mat_global * current_alpha_p_from_state

          // alpha_p_padded để nhân với ma trận 3x3 C_b_n0_mat_global
          // eekf_value current_alpha_p_padded[3] = {current_alpha_p_from_state[0], current_alpha_p_from_state[1], 0.0};
          // eekf_value predicted_beta_p_3D[3];
          // mat3x3_vec3x1_mul(C_b_n0_mat_global, current_alpha_p_padded, predicted_beta_p_3D);
          // Serial.printf("beta_p_from_gps_actual: %.3f, %.3f\n", beta_p_from_gps_actual[0], beta_p_from_gps_actual[1]);
          // Serial.printf("predicted_beta_p_3D: %.3f, %.3f\n", predicted_beta_p_3D[0], predicted_beta_p_3D[1]);
          // Vector đo lường thực tế (z) cho EKF (chỉ lấy X,Y)
          *EEKF_MAT_EL(z, 0, 0) = beta_p_from_gps_actual[0];
          *EEKF_MAT_EL(z, 1, 0) = beta_p_from_gps_actual[1];
          *EEKF_MAT_EL(z, 2, 0) = g_yaw_body;    //- *EEKF_MAT_EL(*ctx.x, 4, 0); // Sự khác biệt yaw giữa GPS và EKF
          *EEKF_MAT_EL(z, 3, 0) = g_groundSpeed_filtered; //- sqrt(ekf_av_x * ekf_av_x + ekf_av_y * ekf_av_y);
          // Cập nhật ma trận R (nhiễu đo lường) - giờ là 2x2
          // Bạn cần xác định giá trị noise này, có thể phụ thuộc vào HDOP GPS.
          // Ví dụ: update_R_matrix(g_hdop); // Bạn cần tự viết hàm này
          // Nếu không, R sẽ giữ giá trị khởi tạo.
          // update_R_matrix(); // Gọi hàm này nếu bạn có
          *EEKF_MAT_EL(R, 0, 0) = pow(g_std_dev_gps_pos, 2);
          *EEKF_MAT_EL(R, 1, 1) = pow(g_std_dev_gps_pos, 2);
          *EEKF_MAT_EL(R, 2, 2) = pow(g_std_dev_imu_yaw, 2);
          *EEKF_MAT_EL(R, 3, 3) = pow(g_std_dev_gps_speed, 2);
          // double mahalanobis_squared = 0.0;
          // bool is_gps_outlier = check_mahalanobis_distance(&ctx, &z, &R, &mahalanobis_squared); // z giờ là innovation

          // Tạm thời bỏ ghi đè cho mục đích gỡ lỗi, nhưng hãy nhớ bỏ chúng trong sản phẩm cuối cùng
          // is_gps_outlier=false;
          // attempt_gps_update=true;
          // Serial.printf("EKF ap_x: %.3f, ekf_ap_y: %.3f, ekf_av_x: %.3f, ekf_av_y: %.3f\n",
          //               ekf_ap_x, ekf_ap_y, ekf_av_x, ekf_av_y);
          // Serial.printf("g_yaw(deg): %.3f, ekf_yaw_rad(deg): %.3f\n", degrees(g_yaw), degrees(ekf_yaw_rad));
          // Serial.printf("g_latitude: %.7f, g_longitude: %.7f\n", g_latitude, g_longitude);
          // Serial.printf("gps_x: %.3f, gps_y: %.3f\n", gps_x, gps_y);
          // Serial.printf("z0: %.3f, z1: %.3f, z2: %.3f, z3: %.3f\n",
          //               *EEKF_MAT_EL(z, 0, 0),
          //               *EEKF_MAT_EL(z, 1, 0),
          //               *EEKF_MAT_EL(z, 2, 0),
          //               *EEKF_MAT_EL(z, 3, 0));
          // Serial.printf("R0: %.3f, R1: %.3f, R2: %.3f, R3: %.3f\n",
          //               *EEKF_MAT_EL(R, 0, 0),
          //               *EEKF_MAT_EL(R, 1, 1),
          //               *EEKF_MAT_EL(R, 2, 2),
          //               *EEKF_MAT_EL(R, 3, 3));
          // delay(30000);
          // print_csv_data_2();
          eekf_correct(&ctx, &z, &R);
          // g_ekf_operation_status = EKF_STATUS_PREDICT_UPDATE_GPS;

          ekf_ap_x = double(*EEKF_MAT_EL(*ctx.x, 0, 0));
          ekf_ap_y = double(*EEKF_MAT_EL(*ctx.x, 1, 0));
          ekf_av_x = double(*EEKF_MAT_EL(*ctx.x, 2, 0));
          ekf_av_y = double(*EEKF_MAT_EL(*ctx.x, 3, 0));
          ekf_yaw_rad = double(*EEKF_MAT_EL(*ctx.x, 4, 0));
          ekf_ba_x = double(*EEKF_MAT_EL(*ctx.x, 5, 0));
          ekf_ba_y = double(*EEKF_MAT_EL(*ctx.x, 6, 0));
          ekf_bg_z = double(*EEKF_MAT_EL(*ctx.x, 7, 0));
          ekf_EKF_Lat = lat_origin + (ekf_ap_y / METERS_PER_DEG_LAT);
          ekf_EKF_Lon = lon_origin + (ekf_ap_x / METERS_PER_DEG_LON);

          // print_csv_data(zupt_was_activated, dT); // In dữ liệu CSV
          print_csv_data_2();
          
          
          // Serial.printf("Time: %ld, EKF: x=%.3f, y=%.3f, GPS: x=%.3f, y=%.3f, Error: %.3f\n",
          //               millis(), ekf_ap_x, ekf_ap_y, gps_x, gps_y,
          //               sqrt(pow(ekf_ap_x - gps_x, 2) + pow(ekf_ap_y - gps_y, 2)));
          // Serial.printf("Vel: EKF_x=%.3f, EKF_y=%.3f, GPS_speed=%.3f, Bias_x=%.3f, Bias_y=%.3f\n",
          //               ekf_av_x, ekf_av_y, g_groundSpeed, ekf_ba_x, ekf_ba_y);
          // Serial.printf("Yaw_body: %.3f, ekf_yaw_rad(deg): %.3f\n",g_yaw, degrees(ekf_yaw_rad));
        }
      }
    }
  }

  // --- Tích hợp ZUPT (Zero-velocity UPdaTe) ---
  // (Đã được comment trong code của bạn, giữ nguyên trạng thái này)
  // Nếu muốn ZUPT hoạt động, logic ZUPT sẽ ở đây (không phụ thuộc GPS Update)
  // (ekf_handler.h cần is_stationary và các hằng số liên quan)
  

  // if (origin_set)
  // {
  //   // --- Áp dụng giả định "xe không lùi" ---
  //   // (Logic này được đặt sau EKF Update/ZUPT để nó là bước cuối cùng)
  //   eekf_value current_estimated_nav_vx = *EEKF_MAT_EL(*ctx.x, 2, 0);
  //   eekf_value current_estimated_nav_vy = *EEKF_MAT_EL(*ctx.x, 3, 0);
  //   eekf_value current_estimated_yaw_rad_for_v_check = *EEKF_MAT_EL(*ctx.x, 4, 0);

  //   // Vận tốc tiến/lùi theo trục X_body (chuyển từ Nav sang Body)
  //   // (Nav: X_East, Y_North; Body: X_Forward, Y_Left)
  //   // Vx_body = Vx_nav * sin(Yaw) + Vy_nav * cos(Yaw)
  //   eekf_value current_v_forward = current_estimated_nav_vx * sin(current_estimated_yaw_rad_for_v_check) +
  //                                  current_estimated_nav_vy * cos(current_estimated_yaw_rad_for_v_check);

  //   const eekf_value REVERSE_SPEED_THRESHOLD = -0.05; // Ngưỡng âm để xác định lùi (ví dụ: -0.05 m/s)

  //   // Chỉ áp dụng ép về 0 nếu tốc độ âm lớn hơn ngưỡng
  //   if (current_v_forward < REVERSE_SPEED_THRESHOLD)
  //   {
  //     // Ép vận tốc về 0 trong nav frame
  //     *EEKF_MAT_EL(*ctx.x, 2, 0) = 0.0; // alpha_v_x
  //     *EEKF_MAT_EL(*ctx.x, 3, 0) = 0.0; // alpha_v_y
  //     // Có thể giảm độ không chắc chắn (P) của vận tốc nếu ép nó về 0
  //     *EEKF_MAT_EL(*ctx.P, 2, 2) *= 0.1; // Giảm hiệp phương sai
  //     *EEKF_MAT_EL(*ctx.P, 3, 3) *= 0.1;
  //     // Đặt các hiệp phương sai chéo về 0 (để thể hiện vận tốc được đặt cứng)
  //     *EEKF_MAT_EL(*ctx.P, 0, 2) = 0.0;
  //     *EEKF_MAT_EL(*ctx.P, 2, 0) = 0.0;
  //     *EEKF_MAT_EL(*ctx.P, 1, 2) = 0.0;
  //     *EEKF_MAT_EL(*ctx.P, 2, 1) = 0.0;
  //     *EEKF_MAT_EL(*ctx.P, 0, 3) = 0.0;
  //     *EEKF_MAT_EL(*ctx.P, 3, 0) = 0.0;
  //     *EEKF_MAT_EL(*ctx.P, 1, 3) = 0.0;
  //     *EEKF_MAT_EL(*ctx.P, 3, 1) = 0.0;
  //   }
  // }
  if (!origin_set)
  {

    initialize_ekf_parameters();

    current_data_index = 1;

    delay(5000); // Chờ một chút để đảm bảo dữ liệu đã được in ra
    // print_csv_header();
    print_csv_header_2();
    *EEKF_MAT_EL(R, 0, 0) = pow(g_std_dev_gps_pos, 2);
    *EEKF_MAT_EL(R, 1, 1) = pow(g_std_dev_gps_pos, 2);
    *EEKF_MAT_EL(R, 2, 2) = pow(g_std_dev_imu_yaw, 2);
    *EEKF_MAT_EL(R, 3, 3) = pow(g_std_dev_gps_speed, 2);
    last_gps_read_time_us = 0;
    last_imu_read_time_us = 0;
    current_loop_time_us = 0; // Reset current loop time
    delta_t_gps_us = 0;
    delta_t_us = 0;
    after_init = micros(); // Lưu thời điểm khởi tạo để tính toán thời gian trôi qua
    g_latitude = lat_origin;
    g_longitude = lon_origin;
    g_yaw = IMU_Yaw_deg[0]; // Giả sử IMU_Yaw_deg[0] là giá trị ban đầu
    g_yaw_body = g_yaw;
    METERS_PER_DEG_LON = 111320.0 * cos(radians(lat_origin));

    gps_history[0].latitude = lat_origin;
    gps_history[0].longitude = lon_origin;
    gps_history[0].local_x = 0;
    gps_history[0].local_y = 0;
    gps_count = 1;

    ekf_ap_x = double(*EEKF_MAT_EL(*ctx.x, 0, 0));
    ekf_ap_y = double(*EEKF_MAT_EL(*ctx.x, 1, 0));
    ekf_av_x = double(*EEKF_MAT_EL(*ctx.x, 2, 0));
    ekf_av_y = double(*EEKF_MAT_EL(*ctx.x, 3, 0));
    ekf_yaw_rad = double(*EEKF_MAT_EL(*ctx.x, 4, 0));
    ekf_ba_x = double(*EEKF_MAT_EL(*ctx.x, 5, 0));
    ekf_ba_y = double(*EEKF_MAT_EL(*ctx.x, 6, 0));
    ekf_bg_z = double(*EEKF_MAT_EL(*ctx.x, 7, 0));
    ekf_EKF_Lat = lat_origin + (ekf_ap_y / METERS_PER_DEG_LAT);
    ekf_EKF_Lon = lon_origin + (ekf_ap_x / METERS_PER_DEG_LON);
    print_csv_data_2(); // In dữ liệu CSV ban đầu
  }
  delayMicroseconds(100);
}
