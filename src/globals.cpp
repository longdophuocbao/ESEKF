#include "globals.h"

const uint8_t RXD2 = 16;
const uint8_t TXD2 = 17;
unsigned long after_init = 0;
float LPF_ALPHA = 0.094;//0.064;
const double GPS_MAHALANOBIS_THRESHOLD = 13.277;

const double ZUPT_VEL_COVARIANCE_REDUCTION = 0.001;

const float MAX_STATIONARY_SPEED_KMPH = 0.15;
float MAX_STATIONARY_OMEGA_RADS_ZUPT = 0.05;
const float MAX_STATIONARY_ACCEL_MAGNITUDE = 0.5;
const unsigned long STATIONARY_DETECT_DURATION_MS = 500;
const float G_A_BODY_ZERO_THRESHOLD = 0.05f; // m/s^2, ví dụ: nếu |g_ax_body| < 0.05 thì coi như 0

uint16_t SAT_COUNT_MIN_R = 15;   // Ngưỡng vệ tinh tối thiểu để có R tốt hơn
uint16_t SAT_COUNT_IDEAL_R = 20; // Ngưỡng vệ tinh lý tưởng
float HDOP_IDEAL_R = 0.5;        // Ngưỡng HDOP lý tưởng
float HDOP_ACCEPTABLE_R = 1.5;   // Ngưỡng HDOP chấp nhận được
float HDOP_POOR_R = 10;          // Ngưỡng HDOP kém

const uint16_t GPS_QUALITY_MIN_SATS = 4; // Yêu cầu ít nhất 4 vệ tinh cho một fix cơ bản
const float GPS_QUALITY_MAX_HDOP = 3.0f; // HDOP lớn hơn 2 thường không đáng tin cậy

float zs_accel_x_buffer[ZUPT_WINDOW_SIZE];
float zs_accel_y_buffer[ZUPT_WINDOW_SIZE];
float zs_omega_z_buffer[ZUPT_WINDOW_SIZE];
int zs_buffer_index = 0;
bool zs_buffer_full = false;

const float ZUPT_ACCEL_STD_DEV_THRESHOLD = 0.1;
const float ZUPT_OMEGA_STD_DEV_THRESHOLD = 0.25;

unsigned long stationary_start_time = 0;
bool is_currently_stationary_flag = false;

ModbusMaster node;
HardwareSerial RS485(1); // Giả sử bạn muốn khởi tạo nó ở đây
WTGAHRS3_485 sensor(node);

eekf_context ctx;
uint16_t g_svnum = 0;
double g_ax_body = 0.0;
double g_ay_body = 0.0;
double g_omega_body = 0.0;
double g_yaw_body = 0.0;
double g_yaw = 0.0;
double g_roll = 0.0;
double g_pitch = 0.0;
double g_longitude = 0.0;
double g_latitude = 0.0;
double g_pdop = 99.0;
double g_hdop = 99.0;
double g_vdop = 99.0;
double g_groundSpeed = 0.0;
double g_accelX = 0.0;
double g_accelY = 0.0;
double g_accelZ = 0.0;
double g_angularVelZ = 0.0;

double g_ax_bias = 0.0;
double g_ay_bias = 0.0;
double g_omega_bias = 0.0;

double g_ax_filtered = 0.0;
double g_ay_filtered = 0.0;
double g_omega_filtered_rads = 0.0;
double g_groundSpeed_filtered = 0.0;
double g_groundSpeed_filtered_kmh_state = 0.0;

// eekf_value g_sigma_accel_noise = 20.80;        // Tùy thuộc vào datasheet IMU của bạn0.07
// eekf_value g_sigma_gyro_noise = 1.5;        // Tùy thuộc vào datasheet IMU của bạn
// eekf_value g_sigma_accel_bias_walk = 10.9; // Tùy thuộc vào độ ổn định của cảm biến
// eekf_value g_sigma_gyro_bias_walk = 0.02;

// double g_std_dev_gps_pos = 2.2;
// double g_std_dev_imu_yaw = 1.7;
// double g_std_dev_gps_speed = 0.5;

eekf_value g_sigma_accel_noise = 0.06;      // 0.0487547638220045; // Tùy thuộc vào datasheet IMU của bạn0.07
eekf_value g_sigma_gyro_noise = 0.005;   // Tùy thuộc vào datasheet IMU của bạn
eekf_value g_sigma_accel_bias_walk = 0.007; // Tùy thuộc vào độ ổn định của cảm biến
eekf_value g_sigma_gyro_bias_walk = 0.00005;

double g_std_dev_gps_pos = 0.12;   // 0.8;
double g_std_dev_imu_yaw = 0.02; // 0.017;
double g_std_dev_gps_speed = 0.02;//0.051;

double Turnthresshold = 0.5; // Ngưỡng góc quay để kích hoạt xoay hay thẳng;
double beta_p_from_gps_actual_0 = 0;
double beta_p_from_gps_actual_1 = 0;

double lat_origin = 0.0;
double lon_origin = 0.0;
bool origin_set = false;
const double METERS_PER_DEG_LAT = 111132.954;
double METERS_PER_DEG_LON = 0.0;

// Các biến cho việc thiết lập gốc tọa độ GPS ban đầu
const float ORIGIN_SETUP_MAX_HDOP = 1.8f; // Ví dụ: HDOP phải dưới 1.5
const int ORIGIN_SETUP_NUM_SAMPLES = 100; // Số lượng mẫu cần thu thập
int origin_setup_sample_count = 0;
double origin_setup_lat_sum = 0.0;
double origin_setup_lon_sum = 0.0;
double averaged_lat_origin = 0.0;
int num_gps_points_collected = 0; // Definition for the global variable
int gps_history_idx = 0;          // Definition and initialization for gps_history_idx
double averaged_lon_origin = 0.0;
GpsPoint gps_history[MAX_GPS_HISTORY_POINTS]; // Definition for gps_history array
int current_data_index = 0;
eekf_value ekf_ap_x = 0.0;
eekf_value ekf_ap_y = 0.0;
eekf_value ekf_av_x = 0.0;
eekf_value ekf_av_y = 0.0;
eekf_value ekf_yaw_rad = 0.0;
eekf_value ekf_ba_x = 0.0;
eekf_value ekf_ba_y = 0.0;
eekf_value ekf_bg_z = 0.0;
// Thêm các biến cho trạng thái EKF mới
eekf_value ekf_EKF_Lat = 0.0;
eekf_value ekf_EKF_Lon = 0.0;
double gps_x = 0.0;
double gps_y = 0.0;

bool zupt_was_activated = true;

SynchronizedGpsData gps_data_local;
SynchronizedImuAttitudeData imu_att_data;

bool isIMUDataValid = false;
bool isGPSDataValid = false; // Biến toàn cục để theo dõi tính hợp lệ của dữ liệu GPS

unsigned long previous_ekf_time_us = 0; // Thời điểm EKF (dựa trên IMU) chạy lần trước
// double R_noise_global[EKF_M * EKF_M]; // Khởi tạo ma trận R toàn cục
EkfOperationStatus g_ekf_operation_status = EKF_STATUS_NONE; // Khởi tạo trạng thái EKF

// Các hằng số cho mô hình
eekf_value dT = 0.03; // Thời gian bước (giây)

// Định nghĩa số lượng trạng thái và phép đo
const uint8_t NUM_STATES = 8;
const uint8_t NUM_MEASUREMENTS = 4;

// Các biến toàn cục cho EKF (đã được khai báo ở trên, chỉ cần đảm bảo nhất quán)
// eekf_value ekf_ap_x, ekf_ap_y, ekf_av_x, ekf_av_y, ekf_yaw_rad, ekf_ba_x, ekf_ba_y, ekf_bg_z;
// eekf_value ekf_EKF_Lat, ekf_EKF_Lon;
// bool zupt_was_activated;
// SynchronizedGpsData gps_data_local;
// SynchronizedImuAttitudeData imu_att_data;
// Độ lệch chuẩn nhiễu đo lường (measurement noise standard deviation)
// (Ước tính độ chính xác của các cảm biến)
eekf_value s_gps_x = g_std_dev_gps_pos; // GPS x (mét)
eekf_value s_gps_y = g_std_dev_gps_pos; // GPS y (mét)

// Loại bỏ các biến vận tốc GPS không còn được sử dụng trong mô hình 2 phép đo
// double gps_x = 0.0, gps_y = 0.0; // Giữ lại nếu vẫn dùng cho mục đích khác

// --- Khai báo toàn cục cho bộ lọc ---
// eekf_context ctx;

// Trạng thái của bộ lọc (8 trạng thái): [alpha_p_x, alpha_p_y, alpha_v_x, alpha_v_y, yaw, b_ax, b_ay, b_gz]^T
EEKF_DECL_MAT_INIT(x, NUM_STATES, 1,
                   0.0, // x[0]: alpha_p_x (PIF Reference Vector X)
                   0.0, // x[1]: alpha_p_y (PIF Reference Vector Y)
                   0.0, // x[2]: alpha_v_x (VIF Reference Vector X)
                   0.0, // x[3]: alpha_v_y (VIF Reference Vector Y)
                   0.0, // x[4]: yaw (Yaw angle in radians)
                   0.1, // x[5]: b_ax (Accelerometer Bias X)
                   0.1, // x[6]: b_ay (Accelerometer Bias Y)
                   0.1  // x[7]: b_gz (Gyroscope Bias Z)
);

// Ma trận hiệp phương sai sai số trạng thái P
EEKF_DECL_MAT_INIT(P, NUM_STATES, NUM_STATES,
                   25.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,    // ap_x
                   0.0, 25.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,    // ap_y
                   0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0,     // av_x
                   0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0,     // av_y
                   0.0, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0,  // yaw
                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0001, 0.0, 0.0,  // b_ax
                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0001, 0.0,  // b_ay
                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.000001 // b_gz
);

// --- Khai báo ma trận đầu vào điều khiển u ---
// Đầu vào điều khiển từ IMU: ax, ay, wz (gia tốc x,y body frame, tốc độ góc z body frame)
EEKF_DECL_MAT_INIT(u, 3, 1,
                   0.0, // u[0]: ax
                   0.0, // u[1]: ay
                   0.0  // u[2]: wz (rad/s)
);

// --- Khai báo ma trận hiệp phương sai nhiễu quá trình Q ---
EEKF_DECL_MAT_INIT(Q, NUM_STATES, NUM_STATES, // 8x8 matrix
                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

// --- Khai báo ma trận đo lường thực tế z ---
// Phép đo: delta_pos_x, delta_pos_y
EEKF_DECL_MAT_INIT(z, NUM_MEASUREMENTS, 1,
                   0.0, // z[0]: delta_pos_x
                   0.0, // z[1]: delta_pos_y
                   0.0,
                   0.0);

// --- Khai báo ma trận hiệp phương sai nhiễu đo lường R ---
// Chỉ có 2 phép đo: delta_pos_x, delta_pos_y
EEKF_DECL_MAT_INIT(R, NUM_MEASUREMENTS, NUM_MEASUREMENTS,
                   0.001, 0, 0, 0, // Pos_x noise (meters^2)
                   0, 0.001, 0, 0, // Pos_y noise (meters^2)
                   0, 0, 0.001, 0,
                   0, 0, 0, 0.5);

void update_Q_matrix(eekf_value current_dT)
{
  g_sigma_accel_noise = 0.07;
  eekf_value accel_noise_variance = g_sigma_accel_noise * g_sigma_accel_noise * current_dT;
  eekf_value gyro_noise_variance = g_sigma_gyro_noise * g_sigma_gyro_noise * current_dT;
  eekf_value accel_bias_walk_variance = g_sigma_accel_bias_walk * g_sigma_accel_bias_walk * current_dT;
  eekf_value gyro_bias_walk_variance = g_sigma_gyro_bias_walk * g_sigma_gyro_bias_walk * current_dT;

  if (zupt_was_activated)
  {
    accel_noise_variance *= 0.1; // Giảm nhiễu vận tốc khi ZUPT
    g_sigma_accel_noise = 0.02;  // Cập nhật lại khi đứng yên
  }

  memset(Q.elements, 0, NUM_STATES * NUM_STATES * sizeof(eekf_value));

  *EEKF_MAT_EL(Q, 0, 0) = 0.25 * accel_noise_variance * (current_dT * current_dT); // Vị trí X
  *EEKF_MAT_EL(Q, 1, 1) = 0.25 * accel_noise_variance * (current_dT * current_dT); // Vị trí Y
  *EEKF_MAT_EL(Q, 2, 2) = accel_noise_variance;                                    // Vận tốc X
  *EEKF_MAT_EL(Q, 3, 3) = accel_noise_variance;                                    // Vận tốc Y
  *EEKF_MAT_EL(Q, 4, 4) = gyro_noise_variance;                                     // Yaw
  *EEKF_MAT_EL(Q, 5, 5) = accel_bias_walk_variance;                                // Bias gia tốc X
  *EEKF_MAT_EL(Q, 6, 6) = accel_bias_walk_variance;                                // Bias gia tốc Y
  *EEKF_MAT_EL(Q, 7, 7) = gyro_bias_walk_variance;                                 // Bias con quay Z
}

void update_R_matrix()
{
  // --- BẮT ĐẦU ĐIỀU CHỈNH MA TRẬN NHIỄU ĐO LƯỜNG R DỰA TRÊN CHẤT LƯỢNG GPS ---
  // Khởi tạo R_matrix về 0 (đặt các phần tử ngoài đường chéo là 0)
  memset(R.elements, 0, NUM_MEASUREMENTS * NUM_MEASUREMENTS * sizeof(eekf_value));

  double base_std_dev_pos = g_std_dev_gps_pos;
  // Tính toán một hệ số điều chỉnh độ lệch chuẩn chung dựa trên chất lượng GPS
  double std_dev_scaling_factor = 1.0;

  // Bước 1: Điều chỉnh dựa trên số lượng vệ tinh
  if (g_svnum < SAT_COUNT_MIN_R)
  {
    std_dev_scaling_factor = 10.0; // std_dev tăng 10 lần
  }
  else if (g_svnum < SAT_COUNT_IDEAL_R) //
  {
    float sat_factor_offset = (float)(SAT_COUNT_IDEAL_R - g_svnum) / (SAT_COUNT_IDEAL_R - SAT_COUNT_MIN_R); // 0 to 1
    // std_dev_scaling_factor là 1.0 ban đầu, sẽ được nhân lên tối đa 3 lần (1.0 * (1+2))
    std_dev_scaling_factor *= (1.0f + sat_factor_offset * 2.0f);
  }
  // Nếu g_svnum >= SAT_COUNT_IDEAL_R, std_dev_scaling_factor vẫn là 1.0 (hoặc giá trị từ SAT_COUNT_MIN_R nếu < min)

  // Bước 2: Tiếp tục điều chỉnh (nhân) hệ số dựa trên HDOP
  if (g_hdop <= HDOP_IDEAL_R)
  {
    // HDOP rất tốt, không thay đổi std_dev_scaling_factor thêm
  }
  else if (g_hdop <= HDOP_ACCEPTABLE_R)
  {
    float hdop_factor_offset = (g_hdop - HDOP_IDEAL_R) / (HDOP_ACCEPTABLE_R - HDOP_IDEAL_R); // 0 to 1
    std_dev_scaling_factor *= (1.0f + hdop_factor_offset * 1.0f);                            // Nhân thêm từ 1.0 đến 2.0
  }
  else if (g_hdop <= HDOP_POOR_R)
  {
    float hdop_factor_offset = (g_hdop - HDOP_ACCEPTABLE_R) / (HDOP_POOR_R - HDOP_ACCEPTABLE_R); // 0 to 1
    std_dev_scaling_factor *= (((std_dev_scaling_factor > 1.5f) ? std_dev_scaling_factor : 1.5f) + hdop_factor_offset * 3.0f);
  }
  else
  {                                 // g_hdop > HDOP_POOR_R
    std_dev_scaling_factor *= 3.0f; // Nhân thêm 3.0
  }
  // Bước 3: Giới hạn factor tối đa
  const float MAX_STD_DEV_FACTOR = 20.0; // Giới hạn std_dev tăng tối đa 20 lần
  std_dev_scaling_factor = fmin(std_dev_scaling_factor, MAX_STD_DEV_FACTOR);

  std_dev_scaling_factor = 1;
  // Bước 4: Cập nhật các phần tử đường chéo của ma trận R
  *EEKF_MAT_EL(R, 0, 0) = pow(base_std_dev_pos * std_dev_scaling_factor, 2); // R[0][0] cho vị trí X
  *EEKF_MAT_EL(R, 1, 1) = pow(base_std_dev_pos * std_dev_scaling_factor, 2); // R[1][1] cho vị trí Y
}

// Define and initialize C_b_n0_mat_global in globals.cpp
eekf_value C_b_n0_mat_global[9] = {
    1.0, 0.0, 0.0, // Column 0
    0.0, 1.0, 0.0, // Column 1
    0.0, 0.0, 1.0  // Column 2
};

// Hàm chuyển Quaternion sang Yaw (radian)
eekf_value quaternion_to_yaw_rad(const Quaternion &q)
{
  // Công thức chuyển Quaternion sang Yaw (từ q_w, q_x, q_y, q_z)
  // Giả sử Yaw là quay quanh trục Z
  eekf_value test = q.x * q.y + q.z * q.w;
  if (test > 0.499)
  { // singularity at north pole
    return 2 * atan2(q.x, q.w);
  }
  if (test < -0.499)
  { // singularity at south pole
    return -2 * atan2(q.x, q.w);
  }
  eekf_value sqx = q.x * q.x;
  eekf_value sqy = q.y * q.y;
  eekf_value sqz = q.z * q.z;
  return atan2(2 * q.y * q.w - 2 * q.x * q.z, 1 - 2 * sqy - 2 * sqz);
}
