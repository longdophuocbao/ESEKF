#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>
#include <ModbusMaster.h>
#include <WTGAHRS3_485.h>
#include "eekf.h"
#include "imu_gps_data_27062025.h" // Bao gồm file dữ liệu mẫu

// ---------------- Cấu hình EKF ----------------
// Chọn độ chính xác cho số thực dấu phẩy động
// #define double double // Đã định nghĩa trong main.cpp trước khi include tinyekf.h

extern const uint8_t RXD2;
extern const uint8_t TXD2;

extern float LPF_ALPHA;
extern const double GPS_MAHALANOBIS_THRESHOLD;

extern const double ZUPT_VEL_COVARIANCE_REDUCTION;

// Các hằng số điều chỉnh cho bộ phát hiện đứng yên
extern const float MAX_STATIONARY_SPEED_KMPH;
extern float MAX_STATIONARY_OMEGA_RADS_ZUPT;
extern const float MAX_STATIONARY_ACCEL_MAGNITUDE;
extern const unsigned long STATIONARY_DETECT_DURATION_MS;
extern const float G_A_BODY_ZERO_THRESHOLD; // Ngưỡng để đưa g_ax_body về 0

// Kích thước cửa sổ cho Rolling Standard Deviation
// extern const int ZUPT_WINDOW_SIZE; // Chuyển định nghĩa trực tiếp vào đây
constexpr int ZUPT_WINDOW_SIZE = 50; // Định nghĩa là một hằng số tại thời điểm biên dịch

// Buffers để lưu trữ dữ liệu cho Rolling Standard Deviation
extern float zs_accel_x_buffer[ZUPT_WINDOW_SIZE];
extern float zs_accel_y_buffer[ZUPT_WINDOW_SIZE];
extern float zs_omega_z_buffer[ZUPT_WINDOW_SIZE];
extern int zs_buffer_index;
extern bool zs_buffer_full;

// Ngưỡng độ lệch chuẩn cho phép đứng yên
extern const float ZUPT_ACCEL_STD_DEV_THRESHOLD;
extern const float ZUPT_OMEGA_STD_DEV_THRESHOLD;

// Biến cho việc đếm thời gian đứng yên
extern unsigned long stationary_start_time;
extern bool is_currently_stationary_flag;

// instantiate ModbusMaster object
extern ModbusMaster node;
extern HardwareSerial RS485;
// instantiate WTGAHRS3_485 object, passing the ModbusMaster object
extern WTGAHRS3_485 sensor;

#define SENSOR_SLAVE_ID 0x50

// --- Biến EKF và cảm biến toàn cục ---
extern eekf_context ctx;
extern uint16_t g_svnum;
extern double g_ax_body;
extern double g_ay_body;
extern double g_omega_body;
extern double g_yaw_body;
extern double g_yaw;
extern double g_roll;
extern double g_pitch;
extern double g_longitude;
extern double g_latitude;
extern double g_pdop;
extern double g_hdop;
extern double g_vdop;
extern double g_groundSpeed;
extern double g_accelX;
extern double g_accelY;
extern double g_accelZ;
extern double g_angularVelZ;

extern double g_ax_bias;
extern double g_ay_bias;
extern double g_omega_bias;

extern double g_ax_filtered;
extern double g_ay_filtered;
extern double g_omega_filtered_rads;
extern double g_groundSpeed_filtered;
extern double g_groundSpeed_filtered_kmh_state;

extern eekf_value g_sigma_accel_noise;     // Độ lệch chuẩn nhiễu gia tốc (m/s^2 / sqrt(Hz))
extern eekf_value g_sigma_gyro_noise;      // Độ lệch chuẩn nhiễu con quay hồi chuyển (rad/s / sqrt(Hz))
extern eekf_value g_sigma_accel_bias_walk; // Độ lệch chuẩn nhiễu Random Walk cho bias gia tốc (m/s^2 / sqrt(s))
extern eekf_value g_sigma_gyro_bias_walk;  // Độ lệch chuẩn nhiễu Random Walk cho bias con quay hồi chuyển (rad/s / sqrt(s))

extern int current_data_index;

extern double lat_origin;
extern double lon_origin;
extern bool origin_set;
extern const double METERS_PER_DEG_LAT;
extern double METERS_PER_DEG_LON;

// Các biến cho việc thiết lập gốc tọa độ GPS ban đầu
extern const float ORIGIN_SETUP_MAX_HDOP;
extern const int ORIGIN_SETUP_NUM_SAMPLES;
extern int origin_setup_sample_count;
extern double origin_setup_lat_sum;
extern double origin_setup_lon_sum;
extern double averaged_lat_origin; // Sẽ được sử dụng bởi initialize_ekf_parameters
extern double averaged_lon_origin; // Sẽ được sử dụng bởi initialize_ekf_parameters

extern double gps_x, gps_y, gps_vx, gps_vy;

extern double Turnthresshold; // Ngưỡng góc quay để kích hoạt xoay hay thẳng
// --- Các biến toàn cục để lưu trữ trạng thái ước tính EKF ---
// Vị trí Loci (alpha_p)
extern eekf_value ekf_ap_x;
extern eekf_value ekf_ap_y;

// Vận tốc Loci (alpha_v)
extern eekf_value ekf_av_x;
extern eekf_value ekf_av_y;

// Quaternion thái độ
extern eekf_value ekf_q_w;
extern eekf_value ekf_q_x;
extern eekf_value ekf_q_y;
extern eekf_value ekf_q_z;

// Bias gia tốc kế
extern eekf_value ekf_ba_x;
extern eekf_value ekf_ba_y;

// Bias con quay hồi chuyển Z
extern eekf_value ekf_bg_z;

// Góc Yaw (chuyển đổi từ Quaternion)
extern eekf_value ekf_yaw_rad; // Tên mới để tránh nhầm lẫn với g_yaw từ IMU

extern eekf_value ekf_EKF_Lat;
extern eekf_value ekf_EKF_Lon;

extern bool zupt_was_activated;

extern SynchronizedGpsData gps_data_local;
extern SynchronizedImuAttitudeData imu_att_data;

extern bool isIMUDataValid;
extern bool isGPSDataValid;

extern unsigned long previous_time_arduino_us; // Thời điểm Arduino của vòng lặp trước
// extern double R_noise_global[EKF_M * EKF_M]; // Ma trận R toàn cục để in

extern const uint8_t NUM_STATES;       // x, y, vx, vy, yaw
extern const uint8_t NUM_MEASUREMENTS; // GPS_x, GPS_y, IMU_ax_local, IMU_angular_velocity


// Định nghĩa các trạng thái hoạt động của EKF
enum EkfOperationStatus : uint8_t
{
  EKF_STATUS_NONE = 0,                // Chưa có hoạt động hoặc reset
  EKF_STATUS_PREDICT_ONLY = 1,        // Chỉ thực hiện bước Dự đoán
  EKF_STATUS_PREDICT_UPDATE_ZUPT = 2, // Dự đoán và cập nhật ZUPT (nếu bạn muốn tách riêng)
  EKF_STATUS_PREDICT_UPDATE_GPS = 3,  // Dự đoán và cập nhật GPS thành công
  EKF_STATUS_GPS_REJECTED = 4,        // Dự đoán, phép đo GPS bị Mahalanobis loại bỏ
  EKF_STATUS_GPS_UPDATE_FAILED = 5,   // Dự đoán, cập nhật GPS thất bại (lỗi nội bộ)
  EKF_STATUS_WAITING_IMU = 6,         // Chờ dữ liệu IMU hợp lệ (EKF không chạy)
  EKF_TURNING = 7,                    // Trạng thái đang quay (đang xoay)
  EKF_STRAIGHT = 8                    // Trạng thái đang đi thẳng (kh
};
extern EkfOperationStatus g_ekf_operation_status;
extern unsigned long previous_ekf_time_us;
// Các hằng số cho mô hình
extern eekf_value dT; // Thời gian bước (giây)
extern unsigned long after_init;
// Độ lệch chuẩn nhiễu quá trình (process noise standard deviation)
// (Ước tính mức độ không chắc chắn trong mô hình động học của chúng ta)
extern eekf_value s_x_process;   // Nhiễu vị trí x
extern eekf_value s_y_process;   // Nhiễu vị trí y
extern eekf_value s_vx_process;  // Nhiễu vận tốc x
extern eekf_value s_vy_process;  // Nhiễu vận tốc y
extern eekf_value s_yaw_process; // Nhiễu hướng yaw (radian)

// Độ lệch chuẩn nhiễu đo lường (measurement noise standard deviation)
// (Ước tính độ chính xác của các cảm biến)
extern eekf_value s_gps_x;                // GPS x (mét)
extern eekf_value s_gps_y;                // GPS y (mét)
extern eekf_value s_imu_ax_local;         // IMU gia tốc x cục bộ (m/s^2)
extern eekf_value s_imu_angular_velocity; // IMU vận tốc góc (rad/s)

extern double g_std_dev_gps_pos;
extern double g_std_dev_imu_yaw;
extern double g_std_dev_gps_speed;

extern double beta_p_from_gps_actual_0;
extern double beta_p_from_gps_actual_1;

// Gia tốc thực và vận tốc góc thực (để mô phỏng)
extern eekf_value true_ax_control;               // AGV tăng tốc theo trục X cục bộ
extern eekf_value true_angular_velocity_control; // AGV quay với vận tốc góc nhỏ
// --- Khai báo extern cho các ma trận EKF ---
// Các ma trận này được ĐỊNH NGHĨA trong globals.cpp sử dụng EEKF_DECL_MAT_INIT

// Trạng thái của bộ lọc: x
extern eekf_value x_elements[]; // Mảng dữ liệu cho x
extern eekf_mat x;              // Cấu trúc ma trận x

// Ma trận hiệp phương sai sai số trạng thái P
extern eekf_value P_elements[];
extern eekf_mat P;

// Đầu vào điều khiển u
extern eekf_value u_elements[];
extern eekf_mat u;

// Ma trận hiệp phương sai nhiễu quá trình Q
extern eekf_value Q_elements[];
extern eekf_mat Q;

// Phép đo z
extern eekf_value z_elements[];
extern eekf_mat z;

// Ma trận hiệp phương sai nhiễu đo lường R
extern eekf_value R_elements[];
extern eekf_mat R;

// Các hằng số điều chỉnh R (Đảm bảo các giá trị này được định nghĩa ở nơi khác)
extern uint16_t SAT_COUNT_MIN_R;   // Ngưỡng vệ tinh tối thiểu để có R tốt hơn
extern uint16_t SAT_COUNT_IDEAL_R; // Ngưỡng vệ tinh lý tưởng
extern float HDOP_IDEAL_R;         // Ngưỡng HDOP lý tưởng
extern float HDOP_ACCEPTABLE_R;    // Ngưỡng HDOP chấp nhận được
extern float HDOP_POOR_R;          // Ngưỡng HDOP kém

// Ngưỡng chất lượng GPS cơ bản để xem xét cập nhật EKF
extern const uint16_t GPS_QUALITY_MIN_SATS; // Số vệ tinh tối thiểu
extern const float GPS_QUALITY_MAX_HDOP;    // HDOP tối đa cho phép

void update_Q_matrix(eekf_value current_dT);
void update_R_matrix(); // Hàm cập nhật ma trận R động

#define MAX_GPS_HISTORY_POINTS 250 // Lưu trữ 200 điểm GPS gần nhất (ví dụ)
struct GpsPoint
{
  double latitude;
  double longitude;
  unsigned long timestamp_ms; // Thời điểm nhận dữ liệu GPS
  // Thêm các trường cho vị trí cục bộ đã chuyển đổi
  eekf_value local_x; // Vị trí X cục bộ (m)
  eekf_value local_y; // Vị trí Y cục bộ (m)
};
extern GpsPoint gps_history[MAX_GPS_HISTORY_POINTS];
extern int gps_history_idx;
extern int num_gps_points_collected;    // Total number of points collected (can be less than MAX_GPS_HISTORY_POINTS)
extern eekf_value C_b_n0_mat_global[9]; // Khai báo là mảng eekf_value
// Initial attitude matrix from OBA (global, calculated in main loop or helper)

struct Quaternion
{
  eekf_value w, x, y, z;
  Quaternion() : w(1.0), x(0.0), y(0.0), z(0.0) {}
  Quaternion(eekf_value _w, eekf_value _x, eekf_value _y, eekf_value _z) : w(_w), x(_x), y(_y), z(_z) {}
  void normalize()
  {
    eekf_value mag = sqrt(w * w + x * x + y * y + z * z);
    if (mag > 1e-9)
    {
      w /= mag;
      x /= mag;
      y /= mag;
      z /= mag;
    }
    else
    {
      w = 1.0;
      x = 0.0;
      y = 0.0;
      z = 0.0;
    }
  }
  // Multiply by a vector (rotate a vector by this quaternion)
  // p_x, p_y, p_z: input vector in body frame
  // result_vec: output vector in navigation frame
  void rotate_vector(eekf_value px, eekf_value py, eekf_value pz, eekf_value *result_vec) const
  {
    // Equivalent to q * [0, px, py, pz] * q_inv
    // Where q_inv is the inverse of the quaternion (conjugate for unit quaternions)
    eekf_value q_inv_x = -x, q_inv_y = -y, q_inv_z = -z; // q_inv_w = w

    eekf_value t_x = w * px + y * pz - z * py;
    eekf_value t_y = w * py + z * px - x * pz;
    eekf_value t_z = w * pz + x * py - y * px;
    eekf_value t_w = -x * px - y * py - z * pz;

    result_vec[0] = t_x * w - t_w * x - t_y * z + t_z * y;
    result_vec[1] = t_y * w - t_w * y - t_z * x + t_x * z;
    result_vec[2] = t_z * w - t_w * z - t_x * y + t_y * x;
  }
  Quaternion operator*(const Quaternion &other) const
  {
    return Quaternion(
        w * other.w - x * other.x - y * other.y - z * other.z, // w component
        w * other.x + x * other.w + y * other.z - z * other.y, // x component
        w * other.y - x * other.z + y * other.w + z * other.x, // y component
        w * other.z + x * other.y - y * other.x + z * other.w  // z component
    );
  }
};

eekf_value quaternion_to_yaw_rad(const Quaternion &q);

#endif // GLOBALS_H
