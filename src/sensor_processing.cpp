#include "sensor_processing.h"
#include <math.h> // Cho fabs, sqrt

// Định nghĩa các hàm đã khai báo trong .h
// --- KHAI BÁO BIẾN TOÀN CỤC HOẶC STATIC CẦN THIẾT ---

// Các biến cho bộ đệm tròn

// Biến để tính toán độ lệch chuẩn di động (Moving Standard Deviation)
// Khởi tạo tất cả bằng 0 khi chương trình bắt đầu
double zs_accel_x_sum = 0.0;
double zs_accel_x_sum_sq = 0.0;
double zs_accel_y_sum = 0.0;
double zs_accel_y_sum_sq = 0.0;
double zs_omega_z_sum = 0.0;
double zs_omega_z_sum_sq = 0.0;

void calibrate_sensors()
{
  Serial.println("Bat dau hieu chinh cam bien... Vui long giu yen thiet bi.");
  const int num_samples = 500; // Giảm số lượng mẫu để hiệu chỉnh nhanh hơn
  double ax_sum = 0.0;
  double ay_sum = 0.0;
  double omega_sum = 0.0;
  int valid_samples = 0;

  for (int i = 0; i < num_samples; i++)
  {
    // Giả sử sensor là biến toàn cục hoặc được truyền vào
    AccelerationData accel_calib = sensor.getAccelerationData();
    AngularVelocityData gyro_calib = sensor.getAngularVelocityData();

    if (accel_calib.isDataValid && gyro_calib.isDataValid)
    {
      ax_sum += accel_calib.accelX;
      ay_sum += accel_calib.accelY;
      omega_sum += radians(gyro_calib.angularVelZ); // Chuyển sang radian
      valid_samples++;
    }
    else
    {
      Serial.println("Hieu chinh: Khong doc duoc du lieu cam bien.");
    }
    delay(5); // Chờ một chút giữa các lần đọc
  }

  if (valid_samples > 0)
  {
    g_ax_bias = ax_sum / valid_samples;
    g_ay_bias = ay_sum / valid_samples;
    g_omega_bias = omega_sum / valid_samples;
    Serial.println("Hieu chinh hoan tat!");
    Serial.printf("Do lech da tinh: ax=%.4f, ay=%.4f, omega_rad=%.4f\n", g_ax_bias, g_ay_bias, g_omega_bias);
    delay(2000); // Chờ một chút để người dùng có thể đọc kết quả
  }
  else
  {
    Serial.println("Hieu chinh that bai: Khong co du lieu hop le.");
    g_ax_bias = 0.0; // Đặt lại giá trị mặc định nếu hiệu chỉnh thất bại
    g_ay_bias = 0.0;
    g_omega_bias = 0.0;
  }
  delay(1000);
}

float calculate_std_dev_optimized(double sum, double sum_sq, int count)
{
  if (count <= 1)
    return 0.0; // Tránh chia cho 0 hoặc tính toán không ý nghĩa

  double mean = sum / count;
  double variance = (sum_sq / count) - (mean * mean);

  // Đảm bảo variance không âm do lỗi dấu phẩy động
  if (variance < 0)
    variance = 0.0;

  return sqrt(variance);
}

bool is_stationary(bool p_is_gps_vel_valid_in) // Thêm đối số để kiểm tra tính hợp lệ của vận tốc GPS
{
  // Lấy giá trị cũ từ buffer trước khi ghi đè, nếu buffer đã đầy
  double old_ax = 0.0, old_ay = 0.0, old_omega_z = 0.0;
  if (zs_buffer_full)
  {
    old_ax = zs_accel_x_buffer[zs_buffer_index];
    old_ay = zs_accel_y_buffer[zs_buffer_index];
    old_omega_z = zs_omega_z_buffer[zs_buffer_index];
  }

  // Cập nhật buffers với dữ liệu IMU hiện tại
  zs_accel_x_buffer[zs_buffer_index] = g_ax_body;    // Gia tốc X đã hiệu chỉnh bias
  zs_accel_y_buffer[zs_buffer_index] = g_ay_body;    // Gia tốc Y đã hiệu chỉnh bias
  zs_omega_z_buffer[zs_buffer_index] = g_omega_body; // Vận tốc góc Z đã hiệu chỉnh bias

  // Cập nhật tổng và tổng bình phương cho tính toán độ lệch chuẩn di động
  // Đối với Gia tốc X
  zs_accel_x_sum += g_ax_body;
  zs_accel_x_sum_sq += g_ax_body * g_ax_body;
  if (zs_buffer_full)
  {
    zs_accel_x_sum -= old_ax;
    zs_accel_x_sum_sq -= old_ax * old_ax;
  }

  // Đối với Gia tốc Y
  zs_accel_y_sum += g_ay_body;
  zs_accel_y_sum_sq += g_ay_body * g_ay_body;
  if (zs_buffer_full)
  {
    zs_accel_y_sum -= old_ay;
    zs_accel_y_sum_sq -= old_ay * old_ay;
  }

  // Đối với Vận tốc góc Z
  zs_omega_z_sum += g_omega_body;
  zs_omega_z_sum_sq += g_omega_body * g_omega_body;
  if (zs_buffer_full)
  {
    zs_omega_z_sum -= old_omega_z;
    zs_omega_z_sum_sq -= old_omega_z * old_omega_z;
  }

  // Cập nhật chỉ số buffer và cờ đầy
  zs_buffer_index = (zs_buffer_index + 1) % ZUPT_WINDOW_SIZE;
  if (!zs_buffer_full && zs_buffer_index == 0)
  {
    zs_buffer_full = true;
  }

  // Xác định kích thước buffer hiện tại
  int current_buffer_size = zs_buffer_full ? ZUPT_WINDOW_SIZE : zs_buffer_index;

  // Đảm bảo có đủ dữ liệu trong buffer để tính toán độ lệch chuẩn đáng tin cậy
  if (current_buffer_size < ZUPT_WINDOW_SIZE / 2) // Yêu cầu ít nhất một nửa buffer đầy
  {
    if (origin_set)
    {
      *EEKF_MAT_EL(*ctx.x, 0, 0) = 0.0; // alpha_p_x
      *EEKF_MAT_EL(*ctx.x, 1, 0) = 0.0; // alpha_p_y
      *EEKF_MAT_EL(*ctx.x, 2, 0) = 0.0; // alpha_v_x
      *EEKF_MAT_EL(*ctx.x, 3, 0) = 0.0; // alpha_v_y
    }

    stationary_start_time = 0;
    is_currently_stationary_flag = true;

    return true;
  }

  // --- Tiêu chí 1: Vận tốc tuyến tính từ GPS (nếu có và hợp lệ) ---
  // Sử dụng p_is_gps_vel_valid_in để kiểm soát việc sử dụng dữ liệu GPS
  bool is_low_speed_gps = true; // Mặc định là true nếu không có GPS hợp lệ hoặc không sử dụng GPS
  if (p_is_gps_vel_valid_in)
  {
    is_low_speed_gps = (g_groundSpeed <= MAX_STATIONARY_SPEED_KMPH);
  }

  // Nếu không có GPS hợp lệ, chúng ta sẽ không dựa vào tiêu chí này,
  // hoặc bạn có thể thêm một ngưỡng vận tốc ước tính từ EKF ở đây.

  // --- Tiêu chí 2: Vận tốc góc Z trung bình rất nhỏ (MEAN) ---
  bool is_low_angular_vel_mean = (fabs(g_omega_body) <= MAX_STATIONARY_OMEGA_RADS_ZUPT);

  // --- Tiêu chí 3: Độ lớn của gia tốc tịnh tiến trung bình rất nhỏ (MEAN) ---
  double current_linear_accel_magnitude = sqrt(g_ax_body * g_ax_body + g_ay_body * g_ay_body);
  bool is_low_linear_accel_mean = (current_linear_accel_magnitude <= MAX_STATIONARY_ACCEL_MAGNITUDE);

  // --- Tiêu chí 4: Độ ổn định của Gia tốc (Standard Deviation trong cửa sổ) ---
  float std_dev_ax = calculate_std_dev_optimized(zs_accel_x_sum, zs_accel_x_sum_sq, current_buffer_size);
  float std_dev_ay = calculate_std_dev_optimized(zs_accel_y_sum, zs_accel_y_sum_sq, current_buffer_size);

  bool is_stable_accel_std = (std_dev_ax <= ZUPT_ACCEL_STD_DEV_THRESHOLD) &&
                             (std_dev_ay <= ZUPT_ACCEL_STD_DEV_THRESHOLD);
  // Serial.print("std_dev_ax:");
  // Serial.print(std_dev_ax, 6);
  // Serial.print(",");
  // Serial.print("std_dev_ay:");
  // Serial.println(std_dev_ax, 6);
  // Serial.print(",");
  // Serial.print("is_stable_accel_std:");
  // Serial.println(is_stable_accel_std);

  // --- Tiêu chí 5: Độ ổn định của Vận tốc góc (Standard Deviation trong cửa sổ) ---
  float std_dev_omega_z = calculate_std_dev_optimized(zs_omega_z_sum, zs_omega_z_sum_sq, current_buffer_size);
  bool is_stable_omega_std = (std_dev_omega_z <= ZUPT_OMEGA_STD_DEV_THRESHOLD);
  // Serial.print("std_dev_omega_z:");
  // Serial.print(std_dev_omega_z, 6);
  // Serial.print(",");
  // Serial.print("is_stable_omega_std:");
  // Serial.println(is_stable_omega_std);
  // Tổng hợp tất cả các tiêu chí
  // ZUPT chỉ được kích hoạt nếu TẤT CẢ các tiêu chí đều đúng
  // bool instantly_stationary = is_low_speed_gps &&
  //                             is_low_angular_vel_mean &&
  //                             is_low_linear_accel_mean &&
  //                             is_stable_accel_std &&
  //                             is_stable_omega_std;
  // Serial.printf("is_low_angular_vel_mean %d, is_low_linear_accel_mean %d, is_stable_accel_std %d, is_stable_omega_std %d\n",
  //               is_low_angular_vel_mean, is_low_linear_accel_mean, is_stable_accel_std, is_stable_omega_std);
  bool instantly_stationary = false;
  if (g_svnum > 20)
  {
    instantly_stationary = is_low_speed_gps &&
                           is_low_angular_vel_mean &&
                           is_low_linear_accel_mean &&
                           is_stable_accel_std &&
                           is_stable_omega_std;
  }
  else
  {
    instantly_stationary = is_low_angular_vel_mean &&
                           is_low_linear_accel_mean &&
                           is_stable_accel_std &&
                           is_stable_omega_std;
  }
  // Serial.printf("instantly_stationary:%d, is_low_speed_gps:%d, is_low_angular_vel_mean:%d, is_low_linear_accel_mean:%d, is_stable_accel_std:%d, is_stable_omega_std:%d\n",
  //               instantly_stationary, is_low_speed_gps, is_low_angular_vel_mean, is_low_linear_accel_mean, is_stable_accel_std, is_stable_omega_std);

  if (instantly_stationary)
  {
    if (stationary_start_time == 0)
    {
      stationary_start_time = millis(); // Bắt đầu tính thời gian đứng yên
    }
    // Nếu đã đứng yên đủ thời gian yêu cầu, đặt cờ
    if (millis() - stationary_start_time >= STATIONARY_DETECT_DURATION_MS)
    {
      is_currently_stationary_flag = true;
    }
  }
  else
  {
    // Reset trạng thái nếu không còn đứng yên
    stationary_start_time = 0;
    is_currently_stationary_flag = false;
  }

  return is_currently_stationary_flag;
}
