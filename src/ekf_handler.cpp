#include "ekf_handler.h"
// #include "matrix_operations.h" // Cho các hàm mat_...
// #include "sensor_processing.h" // Cho is_stationary()
#include <math.h>   // Cho fabs, cos, sin, fmod, pow, radians, degrees
#include <string.h> // Cho memset, memcpy

// Định nghĩa các hàm EKF đã khai báo trong .h
// EEKF_DECL_MAT(S_k_mat, 4, 4); // To accumulate for OBA
// bool S_k_initialized = false; // Flag to initialize S_k_mat once
void initialize_ekf_parameters()
{
  int init_data_index = 0;
  Serial.println("Khoi tao cac tham so EKF...");

  // Thiết lập gốc tọa độ GPS (chỉ Lat/Lon vì bỏ qua độ cao)
  // lat_origin = averaged_lat_origin;
  // lon_origin = averaged_lon_origin;
  lat_origin = GPS_Lat[init_data_index];                    // Sử dụng giá trị đầu tiên trong mảng GPS_Lat
  lon_origin = GPS_Lon[init_data_index];                    // Sử dụng giá trị đầu tiên trong mảng GPS_Lon
  METERS_PER_DEG_LON = 111320.0 * cos(radians(lat_origin)); // Sử dụng METERS_PER_DEG_LON đã thống nhất

  Serial.printf("Goc toa do GPS da dat: Lat=%.7f, Lon=%.7f, m/degLon=%.2f\n", lat_origin, lon_origin, METERS_PER_DEG_LON);
  // delay(2000); // Dòng này không còn cần thiết, đã được ghi chú trước đó

  // Khởi tạo các trạng thái nominal ban đầu (x)
  // NUM_STATES = 8:
  // x[0]: alpha_p_x
  // x[1]: alpha_p_y
  // x[2]: alpha_v_x
  // x[3]: alpha_v_y
  // x[4]: yaw
  // x[5]: b_ax
  // x[6]: b_ay
  // x[7]: b_gz

  // Đặt các thành phần ban đầu của trạng thái nominal
  // alpha_p (PIF Reference Vector) ban đầu là 0
  *EEKF_MAT_EL(x, 0, 0) = 0.0; // alpha_p_x
  *EEKF_MAT_EL(x, 1, 0) = 0.0; // alpha_p_y

  // alpha_v (VIF Reference Vector) ban đầu là 0 (giả định vận tốc ban đầu bằng 0)
  *EEKF_MAT_EL(x, 2, 0) = 0.0; // alpha_v_x
  *EEKF_MAT_EL(x, 3, 0) = 0.0; // alpha_v_y

  // Yaw ban đầu là 0 (hướng về phía Bắc)
  *EEKF_MAT_EL(x, 4, 0) = radians(IMU_Yaw_deg[init_data_index]); // yaw (rad)

  // Biases ban đầu là 0
  *EEKF_MAT_EL(x, 5, 0) = 0.00;  // b_ax
  *EEKF_MAT_EL(x, 6, 0) = 0.00;  // b_ay
  *EEKF_MAT_EL(x, 7, 0) = 0.000; // b_gz

  // Khởi tạo ngữ cảnh EKF
  eekf_init(&ctx, &x, &P, transition, measurement, NULL);
  Serial.printf("ekf init: apx %.2f, apy %.2f, avx %.2f, avy %.2f, yaw %.2f rad (%.2f deg), bax %.6f, bay %.6f, bgz %.6f \n",
                *EEKF_MAT_EL(x, 0, 0), *EEKF_MAT_EL(x, 1, 0),
                *EEKF_MAT_EL(x, 2, 0), *EEKF_MAT_EL(x, 3, 0),
                *EEKF_MAT_EL(x, 4, 0), degrees(*EEKF_MAT_EL(x, 4, 0)),
                *EEKF_MAT_EL(x, 5, 0), *EEKF_MAT_EL(x, 6, 0),
                *EEKF_MAT_EL(x, 7, 0));
  // Khởi tạo ma trận C_b_n0_mat_global là đơn vị (3x3 Identity Matrix)
  // Đây là ma trận thái độ ban đầu (C_b^n(0)) được xác định bởi OBA.
  // Khởi tạo là đơn vị cho đến khi OBA cập nhật nó.
  memset(C_b_n0_mat_global, 0, sizeof(C_b_n0_mat_global));
  C_b_n0_mat_global[0] = (eekf_value)1.0; // m00
  C_b_n0_mat_global[4] = (eekf_value)1.0; // m11
  C_b_n0_mat_global[8] = (eekf_value)1.0; // m22

  // Khởi tạo ma trận S_k cho OBA
  // S_k_mat là một eekf_mat struct, cần truy cập .elements
  // memset(S_k_mat.elements, 0, sizeof(eekf_value) * 16); // 4x4 = 16 phần tử
  // S_k_initialized = true;                               // Đánh dấu đã khởi tạo S_k_mat

  origin_set = true;
  Serial.println("EKF da khoi tao.");
  num_gps_points_collected = 1;
  gps_history_idx = 1;
}

/**
 * @brief Hàm dự đoán trạng thái (transition function) của EKF.
 * Dự đoán trạng thái tiếp theo (xp) và Jacobian (Jf) của hàm chuyển trạng thái.
 *
 * @param xp Con trỏ tới ma trận trạng thái dự đoán (8x1).
 * @param Jf Con trỏ tới ma trận Jacobian của hàm chuyển trạng thái (8x8).
 * @param x Con trỏ tới ma trận trạng thái hiện tại (8x1).
 * @param u Con trỏ tới ma trận đầu vào điều khiển (3x1, ax, ay, wz).
 * @param userData Dữ liệu tùy chỉnh (không sử dụng ở đây).
 * @return eekf_return Trạng thái trả về của EKF.
 */
eekf_return transition(eekf_mat *xp, eekf_mat *Jf, eekf_mat const *x,
                       eekf_mat const *u, void *userData)
{
  eekf_value delta_t_sec = dT;

  // State trước đó
  eekf_value pos_x = *EEKF_MAT_EL(*x, 0, 0);
  eekf_value pos_y = *EEKF_MAT_EL(*x, 1, 0);
  eekf_value vel_x = *EEKF_MAT_EL(*x, 2, 0);
  eekf_value vel_y = *EEKF_MAT_EL(*x, 3, 0);
  eekf_value yaw = *EEKF_MAT_EL(*x, 4, 0);
  eekf_value ba_x = *EEKF_MAT_EL(*x, 5, 0);
  eekf_value ba_y = *EEKF_MAT_EL(*x, 6, 0);
  eekf_value bg_z = *EEKF_MAT_EL(*x, 7, 0);

  // Input
  eekf_value ax_raw = *EEKF_MAT_EL(*u, 0, 0);
  eekf_value ay_raw = *EEKF_MAT_EL(*u, 1, 0);
  eekf_value wz_raw = *EEKF_MAT_EL(*u, 2, 0);

  // Hiệu chỉnh bias
  eekf_value ax_corr = ax_raw - ba_x;
  eekf_value ay_corr = ay_raw - ba_y;
  eekf_value wz_corr = wz_raw - bg_z;

  // Cập nhật yaw
  eekf_value next_yaw = yaw + wz_corr * delta_t_sec;
  next_yaw = fmod(next_yaw + M_PI, 2.0 * M_PI);
  if (next_yaw < 0)
    next_yaw += 2.0 * M_PI;
  next_yaw -= M_PI;

  // cos/sin
  eekf_value cy = cos(yaw);
  eekf_value sy = sin(yaw);

  // Chuyển gia tốc body sang nav frame
  eekf_value ax_nav = ax_corr * sy + ay_corr * cy;
  eekf_value ay_nav = ax_corr * cy - ay_corr * sy;

  // Update vận tốc nav frame
  eekf_value next_vx = vel_x + ax_nav * delta_t_sec;
  eekf_value next_vy = vel_y + ay_nav * delta_t_sec;

  // Update vị trí
  eekf_value next_px = pos_x + vel_x * delta_t_sec;
  eekf_value next_py = pos_y + vel_y * delta_t_sec;

  // Gán state dự đoán
  *EEKF_MAT_EL(*xp, 0, 0) = next_px;
  *EEKF_MAT_EL(*xp, 1, 0) = next_py;
  *EEKF_MAT_EL(*xp, 2, 0) = next_vx;
  *EEKF_MAT_EL(*xp, 3, 0) = next_vy;
  *EEKF_MAT_EL(*xp, 4, 0) = next_yaw;
  *EEKF_MAT_EL(*xp, 5, 0) = ba_x;
  *EEKF_MAT_EL(*xp, 6, 0) = ba_y;
  *EEKF_MAT_EL(*xp, 7, 0) = bg_z;

  // Jacobian 8x8
  for (uint8_t r = 0; r < 8; ++r)
    for (uint8_t c = 0; c < 8; ++c)
      *EEKF_MAT_EL(*Jf, r, c) = (r == c) ? 1.0 : 0.0;

  // d(px)/d(vx), d(py)/d(vy)
  *EEKF_MAT_EL(*Jf, 0, 2) = delta_t_sec;
  *EEKF_MAT_EL(*Jf, 1, 3) = delta_t_sec;

  // d(vx)/d(yaw), d(vy)/d(yaw)
  eekf_value dax_dyaw = ax_corr * cy - ay_corr * sy;
  eekf_value day_dyaw = -ax_corr * sy - ay_corr * cy;
  *EEKF_MAT_EL(*Jf, 2, 4) = dax_dyaw * delta_t_sec;
  *EEKF_MAT_EL(*Jf, 3, 4) = day_dyaw * delta_t_sec;

  // d(vx)/d(ba_x), d(vx)/d(ba_y)
  *EEKF_MAT_EL(*Jf, 2, 5) = -sy * delta_t_sec;
  *EEKF_MAT_EL(*Jf, 2, 6) = -cy * delta_t_sec;
  *EEKF_MAT_EL(*Jf, 3, 5) = -cy * delta_t_sec;
  *EEKF_MAT_EL(*Jf, 3, 6) = sy * delta_t_sec;

  // d(yaw)/d(bg_z)
  *EEKF_MAT_EL(*Jf, 4, 7) = -delta_t_sec;

  return eEekfReturnOk;
}
/* eekf_return transition(eekf_mat *xp, eekf_mat *Jf, eekf_mat const *x,
                       eekf_mat const *u, void *userData)
{
  // --- Lấy trạng thái ---
  eekf_value prev_ap_x = *EEKF_MAT_EL(*x, 0, 0);
  eekf_value prev_ap_y = *EEKF_MAT_EL(*x, 1, 0);
  eekf_value prev_av_x = *EEKF_MAT_EL(*x, 2, 0);
  eekf_value prev_av_y = *EEKF_MAT_EL(*x, 3, 0);
  eekf_value prev_yaw = *EEKF_MAT_EL(*x, 4, 0);
  eekf_value prev_ba_x = *EEKF_MAT_EL(*x, 5, 0);
  eekf_value prev_ba_y = *EEKF_MAT_EL(*x, 6, 0);
  eekf_value prev_bg_z = *EEKF_MAT_EL(*x, 7, 0);

  // --- Lấy dữ liệu IMU ---
  eekf_value input_ax_raw = *EEKF_MAT_EL(*u, 0, 0);
  eekf_value input_ay_raw = *EEKF_MAT_EL(*u, 1, 0);
  eekf_value input_wz_raw = *EEKF_MAT_EL(*u, 2, 0);

  eekf_value delta_t_sec = dT;

  // --- Hiệu chỉnh bias ---
  eekf_value f_b_x_corr = input_ax_raw - prev_ba_x;
  eekf_value f_b_y_corr = input_ay_raw - prev_ba_y;
  eekf_value wz_corr = input_wz_raw - prev_bg_z;

  // --- Cập nhật yaw ---
  eekf_value next_yaw = prev_yaw + wz_corr * delta_t_sec;
  next_yaw = fmod(next_yaw + M_PI, 2.0 * M_PI);
  if (next_yaw < 0.0)
    next_yaw += 2.0 * M_PI;
  next_yaw -= M_PI;
  *EEKF_MAT_EL(*xp, 4, 0) = next_yaw;

  // --- Tính sin/cos ---
  eekf_value c_yaw = cos(prev_yaw);
  eekf_value s_yaw = sin(prev_yaw);
  eekf_value c_th_next = cos(next_yaw);
  eekf_value s_th_next = sin(next_yaw);

  // --- Tính vận tốc body X từ Nav ---
  eekf_value prev_v_body_x = prev_av_x * s_yaw + prev_av_y * c_yaw;
  eekf_value next_v_body_x = prev_v_body_x + f_b_x_corr * delta_t_sec;

  // --- Cập nhật vị trí Nav ---
  if (fabs(wz_corr) > Turnthresshold)
  {
    g_ekf_operation_status = EKF_TURNING; // Đặt trạng thái là đang quay
    eekf_value turn_radius = next_v_body_x / wz_corr;
    *EEKF_MAT_EL(*xp, 0, 0) = prev_ap_x + turn_radius * (s_th_next - s_yaw);
    *EEKF_MAT_EL(*xp, 1, 0) = prev_ap_y + turn_radius * (c_th_next - c_yaw);
  }
  else
  {
    g_ekf_operation_status = EKF_STRAIGHT; // Đặt trạng thái là đang đi thẳng
    *EEKF_MAT_EL(*xp, 0, 0) = prev_ap_x + next_v_body_x * s_yaw * delta_t_sec;
    *EEKF_MAT_EL(*xp, 1, 0) = prev_ap_y + next_v_body_x * c_yaw * delta_t_sec;
  }

  // --- Cập nhật vận tốc Nav ---
  *EEKF_MAT_EL(*xp, 2, 0) = next_v_body_x * s_th_next;
  *EEKF_MAT_EL(*xp, 3, 0) = next_v_body_x * c_th_next;

  // --- Bias giữ nguyên ---
  *EEKF_MAT_EL(*xp, 5, 0) = prev_ba_x;
  *EEKF_MAT_EL(*xp, 6, 0) = prev_ba_y;
  *EEKF_MAT_EL(*xp, 7, 0) = prev_bg_z;

  // --- Tính Jacobian ---
  for (uint8_t r = 0; r < NUM_STATES; ++r)
    for (uint8_t c = 0; c < NUM_STATES; ++c)
      *EEKF_MAT_EL(*Jf, r, c) = (r == c) ? 1.0 : 0.0;

  eekf_value d_next_v_body_x_d_prev_av_x = s_yaw;
  eekf_value d_next_v_body_x_d_prev_av_y = c_yaw;
  eekf_value d_next_v_body_x_d_prev_yaw = prev_av_x * c_yaw - prev_av_y * s_yaw;

  if (fabs(wz_corr) > Turnthresshold)
  {
    eekf_value inv_omega = 1.0 / wz_corr;
    *EEKF_MAT_EL(*Jf, 0, 2) = inv_omega * d_next_v_body_x_d_prev_av_x * (s_th_next - s_yaw);
    *EEKF_MAT_EL(*Jf, 0, 3) = inv_omega * d_next_v_body_x_d_prev_av_y * (s_th_next - s_yaw);
    *EEKF_MAT_EL(*Jf, 0, 4) = inv_omega * (d_next_v_body_x_d_prev_yaw * (s_th_next - s_yaw) + next_v_body_x * (c_th_next - c_yaw));

    *EEKF_MAT_EL(*Jf, 1, 2) = inv_omega * d_next_v_body_x_d_prev_av_x * (c_th_next - c_yaw);
    *EEKF_MAT_EL(*Jf, 1, 3) = inv_omega * d_next_v_body_x_d_prev_av_y * (c_th_next - c_yaw);
    *EEKF_MAT_EL(*Jf, 1, 4) = inv_omega * (d_next_v_body_x_d_prev_yaw * (c_th_next - c_yaw) - next_v_body_x * (s_th_next - s_yaw));
  }
  else
  {
    *EEKF_MAT_EL(*Jf, 0, 2) = d_next_v_body_x_d_prev_av_x * s_yaw * delta_t_sec;
    *EEKF_MAT_EL(*Jf, 0, 3) = d_next_v_body_x_d_prev_av_y * s_yaw * delta_t_sec;
    *EEKF_MAT_EL(*Jf, 0, 4) = (d_next_v_body_x_d_prev_yaw * s_yaw + next_v_body_x * c_yaw) * delta_t_sec;

    *EEKF_MAT_EL(*Jf, 1, 2) = d_next_v_body_x_d_prev_av_x * c_yaw * delta_t_sec;
    *EEKF_MAT_EL(*Jf, 1, 3) = d_next_v_body_x_d_prev_av_y * c_yaw * delta_t_sec;
    *EEKF_MAT_EL(*Jf, 1, 4) = (d_next_v_body_x_d_prev_yaw * c_yaw - next_v_body_x * s_yaw) * delta_t_sec;
  }

  // --- Jacobian vận tốc Nav ---
  *EEKF_MAT_EL(*Jf, 2, 2) = d_next_v_body_x_d_prev_av_x * s_th_next;
  *EEKF_MAT_EL(*Jf, 2, 3) = d_next_v_body_x_d_prev_av_y * s_th_next;
  *EEKF_MAT_EL(*Jf, 2, 4) = d_next_v_body_x_d_prev_yaw * s_th_next + next_v_body_x * c_th_next;

  *EEKF_MAT_EL(*Jf, 3, 2) = d_next_v_body_x_d_prev_av_x * c_th_next;
  *EEKF_MAT_EL(*Jf, 3, 3) = d_next_v_body_x_d_prev_av_y * c_th_next;
  *EEKF_MAT_EL(*Jf, 3, 4) = d_next_v_body_x_d_prev_yaw * c_th_next - next_v_body_x * s_th_next;

  // --- Jacobian bias ---
  *EEKF_MAT_EL(*Jf, 2, 5) = -s_th_next * delta_t_sec;
  *EEKF_MAT_EL(*Jf, 2, 6) = -c_th_next * delta_t_sec;
  *EEKF_MAT_EL(*Jf, 3, 5) = -c_th_next * delta_t_sec;
  *EEKF_MAT_EL(*Jf, 3, 6) = s_th_next * delta_t_sec;

  *EEKF_MAT_EL(*Jf, 4, 7) = -delta_t_sec;

  return eEekfReturnOk;
} */
/* eekf_return transition(eekf_mat *xp, eekf_mat *Jf, eekf_mat const *x,
                       eekf_mat const *u, void *userData)
{
  // --- Lấy các giá trị trạng thái hiện tại ---
  eekf_value prev_ap_x = *EEKF_MAT_EL(*x, 0, 0);
  eekf_value prev_ap_y = *EEKF_MAT_EL(*x, 1, 0);

  eekf_value prev_av_x = *EEKF_MAT_EL(*x, 2, 0);
  eekf_value prev_av_y = *EEKF_MAT_EL(*x, 3, 0);

  eekf_value prev_yaw = *EEKF_MAT_EL(*x, 4, 0);
  eekf_value prev_ba_x = *EEKF_MAT_EL(*x, 5, 0);
  eekf_value prev_ba_y = *EEKF_MAT_EL(*x, 6, 0);
  eekf_value prev_bg_z = *EEKF_MAT_EL(*x, 7, 0);

  // --- Lấy các giá trị đầu vào điều khiển (IMU raw measurements) ---
  eekf_value input_ax_raw = *EEKF_MAT_EL(*u, 0, 0);
  eekf_value input_ay_raw = *EEKF_MAT_EL(*u, 1, 0);
  eekf_value input_wz_raw = *EEKF_MAT_EL(*u, 2, 0);

  eekf_value delta_t_sec = dT; // Khoảng thời gian giữa các lần cập nhật

  // --- Tính toán trạng thái dự đoán xp ---

  // 1. Gia tốc và tốc độ góc đã hiệu chỉnh bias
  eekf_value f_b_x_corr = input_ax_raw - prev_ba_x;
  eekf_value f_b_y_corr = input_ay_raw - prev_ba_y;
  eekf_value wz_corr = input_wz_raw - prev_bg_z;

  // 2. Cập nhật yaw
  eekf_value next_yaw = prev_yaw + wz_corr * delta_t_sec; // wz_corr = 0 nên next_yaw = prev_yaw_from_state

  // Chuẩn hóa góc yaw về khoảng [-PI, PI]
  next_yaw = fmod(next_yaw + M_PI, 2.0 * M_PI);
  if (next_yaw < 0.0)
  {
    next_yaw += 2.0 * M_PI;
  }
  next_yaw -= M_PI;

  *EEKF_MAT_EL(*xp, 4, 0) = next_yaw; // Cập nhật trạng thái Yaw dự đoán

  // 3. KHÔNG chuyển gia tốc từ body frame sang navigation frame
  // Gia tốc sẽ được tích hợp TRỰC TIẾP trong body frame

  // 4. Cập nhật alpha_v (VIF Reference Vector)
  // Vận tốc alpha_v cũng sẽ ở BODY FRAME
  *EEKF_MAT_EL(*xp, 2, 0) = prev_av_x + f_b_x_corr * delta_t_sec;
  *EEKF_MAT_EL(*xp, 3, 0) = prev_av_y + f_b_y_corr * delta_t_sec; // prev_av_y + 0 * dt_sec

  // 5. Cập nhật alpha_p (PIF Reference Vector)
  // Vị trí alpha_p cũng sẽ ở BODY FRAME
  *EEKF_MAT_EL(*xp, 0, 0) = prev_ap_x + prev_av_x * delta_t_sec;
  *EEKF_MAT_EL(*xp, 1, 0) = prev_ap_y + prev_av_y * delta_t_sec;

  // 6. Biases IMU (mô hình Random Walk, giá trị dự đoán bằng giá trị trước đó)
  *EEKF_MAT_EL(*xp, 5, 0) = prev_ba_x;
  *EEKF_MAT_EL(*xp, 6, 0) = prev_ba_y;
  *EEKF_MAT_EL(*xp, 7, 0) = prev_bg_z;

  // --- Tính toán Jacobian của hàm transition (Jf) ---
  // Kích thước: 8x8
  // KHỞI TẠO MA TRẬN ĐƠN VỊ VÀ ĐIỀN CÁC ĐẠO HÀM KHÁC 0
  for (uint8_t r = 0; r < NUM_STATES; ++r)
  {
    for (uint8_t c = 0; c < NUM_STATES; ++c)
    {
      *EEKF_MAT_EL(*Jf, r, c) = (r == c) ? 1.0 : 0.0;
    }
  }

  // Khối con d(alpha_p_next)/d(alpha_v_current) (2x2)
  *EEKF_MAT_EL(*Jf, 0, 2) = delta_t_sec; // d(ap_x_next)/d(av_x)
  *EEKF_MAT_EL(*Jf, 1, 3) = delta_t_sec; // d(ap_y_next)/d(av_y)

  // Khối con d(alpha_v)/d(b_a) (2x2)
  // alpha_v_next = prev_av + (input_ax_raw - prev_ba_x) * dt
  // d(av_x_next)/d(b_ax) = -dt
  // d(av_y_next)/d(b_ay) = -dt
  *EEKF_MAT_EL(*Jf, 2, 5) = -delta_t_sec; // d(av_x_next)/d(b_ax)
  // *EEKF_MAT_EL(*Jf, 2, 6) = 0.0; // d(av_x_next)/d(b_ay) -> đã là 0
  // *EEKF_MAT_EL(*Jf, 3, 5) = 0.0; // d(av_y_next)/d(b_ax) -> đã là 0
  *EEKF_MAT_EL(*Jf, 3, 6) = -delta_t_sec; // d(av_y_next)/d(b_ay)

  // Khối con d(yaw_next)/d(b_gz) (1x1)
  // Vì wz_corr = 0, thì next_yaw = prev_yaw.
  // Nếu next_yaw = prev_yaw, thì d(next_yaw)/d(b_gz) = 0.
  *EEKF_MAT_EL(*Jf, 4, 7) = 0.0; // d(yaw_next)/d(b_gz) = 0 nếu wz_corr = 0

  return eEekfReturnOk;
} */

/**
 * @brief Hàm đo lường (measurement function) của EKF.
 * Tính toán phép đo dự đoán (zp) và Jacobian (Jh) của hàm đo lường.
 * Phép đo (z) là sai số vị trí X, Y (2D) dựa trên phương pháp PIF/OBA.
 *
 * @param zp Con trỏ tới ma trận 4x1 để lưu phép đo dự đoán.
 * @param Jh Con trỏ tới ma trận 2xNUM_STATES (2x8) để lưu Jacobian của hàm đo lường.
 * @param x Con trỏ tới ma trận trạng thái hiện tại (8x1).
 * @param userData Dữ liệu tùy chỉnh (không sử dụng ở đây).
 * @return eekf_return Trạng thái trả về của EKF.
 */
eekf_return measurement(eekf_mat *zp, eekf_mat *Jh, eekf_mat const *x,
                        void *userData)
{
  // Lấy các giá trị trạng thái ước tính hiện tại
  eekf_value current_alpha_p_vec_x = *EEKF_MAT_EL(*x, 0, 0);
  eekf_value current_alpha_p_vec_y = *EEKF_MAT_EL(*x, 1, 0);
  eekf_value current_yaw_est = *EEKF_MAT_EL(*x, 4, 0);
  eekf_value current_av_x_est = *EEKF_MAT_EL(*x, 2, 0);
  eekf_value current_av_y_est = *EEKF_MAT_EL(*x, 3, 0);
  // Serial.printf("Measurement: ap_x=%.5f, ap_y=%.5f, av_x=%.5f, av_y=%.5f, yaw=%.5f\n",
  //               current_alpha_p_vec_x, current_alpha_p_vec_y,
  //               current_av_x_est, current_av_y_est,
  //               degrees(current_yaw_est));

  // 1. Phép đo vị trí dự đoán (từ PIF/OBA)
  // h(x) = C_b^n(0)_k * alpha_p,k (theo Equation 30 của bài báo)
  // C_b_n0_mat_global (ma trận thái độ ban đầu) được tính bởi OBA bên ngoài hàm này.
  // Nó xoay quỹ đạo vị trí IMU (alpha_p) về hệ tọa độ thế giới.

  eekf_value current_alpha_p_padded[3] = {current_alpha_p_vec_x, current_alpha_p_vec_y, 0.0};
  eekf_value predicted_beta_p_3D[3];
  mat3x3_vec3x1_mul(C_b_n0_mat_global, current_alpha_p_padded, predicted_beta_p_3D);

  // Gán các thành phần X, Y của vector đo lường dự đoán (zp)
  *EEKF_MAT_EL(*zp, 0, 0) = predicted_beta_p_3D[0];
  *EEKF_MAT_EL(*zp, 1, 0) = predicted_beta_p_3D[1];
  *EEKF_MAT_EL(*zp, 2, 0) = current_yaw_est;                                                                 // z[2]: yaw dự đoán (lấy trực tiếp từ trạng thái ở bước dự doán)
  *EEKF_MAT_EL(*zp, 3, 0) = sqrt(current_av_x_est * current_av_x_est + current_av_y_est * current_av_y_est); // z[3]: tốc độ tổng hợp dự đoán

  // --- Tính toán Jacobian của hàm đo lường (Jh) ---
  // Kích thước: NUM_MEASUREMENTS x NUM_STATES (2x8)

  // Khởi tạo Jh là ma trận 0
  for (uint8_t r = 0; r < NUM_MEASUREMENTS; ++r)
  {
    for (uint8_t c = 0; c < NUM_STATES; ++c)
    {
      *EEKF_MAT_EL(*Jh, r, c) = 0.0;
    }
  }

  // Đạo hàm của predicted_beta_p theo alpha_p (khối 2x2)
  // Đây là phần 2x2 trên cùng bên trái của ma trận C_b^n(0)_k (tức C_b_n0_mat_global)
  // Vị trí trong Jh: hàng 0,1 (phép đo X,Y), cột 0,1 (alpha_p_x,y)
  *EEKF_MAT_EL(*Jh, 0, 0) = C_b_n0_mat_global[0]; // d(zp_x)/d(ap_x)
  *EEKF_MAT_EL(*Jh, 0, 1) = C_b_n0_mat_global[3]; // d(zp_x)/d(ap_y)
  *EEKF_MAT_EL(*Jh, 1, 0) = C_b_n0_mat_global[1]; // d(zp_y)/d(ap_x)
  *EEKF_MAT_EL(*Jh, 1, 1) = C_b_n0_mat_global[4]; // d(zp_y)/d(ap_y)

  // 2. Đạo hàm của yaw dự đoán (zp[2]) theo trạng thái (yaw_est)
  // zp[2] = x[4]
  // Vị trí trong Jh: hàng 2 (phép đo Yaw), cột 4 (yaw_est)
  *EEKF_MAT_EL(*Jh, 2, 4) = 1.0; // d(zp_2)/d(x_4) = d(yaw_est)/d(yaw_est) = 1.0

  // 3. Đạo hàm của tốc độ dự đoán (zp[3]) theo trạng thái vận tốc (alpha_v_x, alpha_v_y)
  // zp[3] = sqrt(av_x_est^2 + av_y_est^2)
  // Vị trí trong Jh: hàng 3 (phép đo Speed), cột 2 (av_x_est), cột 3 (av_y_est)
  eekf_value current_speed_est = *EEKF_MAT_EL(*zp, 3, 0); // Lấy giá trị tốc độ đã tính cho zp[3]

  if (current_speed_est > 1e-6)
  {                                                                 // Tránh chia cho 0 nếu tốc độ bằng 0
    *EEKF_MAT_EL(*Jh, 3, 2) = current_av_x_est / current_speed_est; // d(zp_3)/d(x_2) = d(speed)/d(av_x)
    *EEKF_MAT_EL(*Jh, 3, 3) = current_av_y_est / current_speed_est; // d(zp_3)/d(x_3) = d(speed)/d(av_y)
  }
  else
  {
    // Nếu tốc độ gần bằng 0, đạo hàm cũng gần bằng 0. Hoặc bạn có thể đặt 0.0.
    // Cần cẩn thận ở đây. Trong thực tế, có thể dùng ngưỡng ZUPT để không cập nhật khi vận tốc 0.
    *EEKF_MAT_EL(*Jh, 3, 2) = 0.0;
    *EEKF_MAT_EL(*Jh, 3, 3) = 0.0;
  }

  // Các đạo hàm khác của Jh (đối với bias) là 0
  // theo công thức H_k của bài báo (H_k = [C_b^n(0)_k O] cho phần vị trí).
  // Phần Yaw và Speed là phép đo trực tiếp từ trạng thái, không liên quan đến bias.

  return eEekfReturnOk;
}

// eekf_mat C_b_n0_mat_global[9]; // Khai báo và khởi tạo ban đầu là ma trận đơn vị

bool ExtendedKalmanFilter(bool *zupt_flag_out, bool p_is_gps_valid)
{
  // Đặt lại trạng thái ở đầu mỗi lần gọi
  *zupt_flag_out = false;
  g_ekf_operation_status = EKF_STATUS_NONE; // Reset trạng thái ở đầu mỗi lần gọi

  // Cập nhật đầu vào của bộ lọc Kalman (u)
  *EEKF_MAT_EL(u, 0, 0) = g_ax_body;              // g_ax_body là gia tốc đã lọc và trừ bias
  *EEKF_MAT_EL(u, 1, 0) = g_ay_body;              // g_ay_body
  *EEKF_MAT_EL(u, 2, 0) = radians(g_angularVelZ); // Vận tốc góc Z thô (rad/s), bias sẽ được trừ trong hàm transition

  eekf_predict(&ctx, &u, &Q);

  g_ekf_operation_status = EKF_STATUS_PREDICT_ONLY;

  // --- BƯỚC CẬP NHẬT (UPDATE) KHI CÓ DỮ LIỆU GPS --
  // Kiểm tra chất lượng GPS
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

    if (attempt_gps_update) // Kiểm tra lại sau khi tính beta_p
    {
      // Bước 2: Lấy alpha_p hiện tại từ trạng thái bộ lọc
      eekf_value current_alpha_p_from_state[2] = {*EEKF_MAT_EL(*ctx.x, 0, 0), *EEKF_MAT_EL(*ctx.x, 1, 0)};

      // Bước 3: Tính toán C_b^n(0)_k (Initial Attitude Matrix) bằng OBA
      // Hàm này sẽ cập nhật C_b_n0_mat_global
      calculate_initial_attitude_OBA(current_alpha_p_from_state, beta_p_from_gps_actual, C_b_n0_mat_global);
      // C_b_n0_mat_global bây giờ đã được tính toán lại bằng OBA.

      // --- Tính toán vector đo lường Z (innovation) cho EKF ---
      // Z_k = beta_p,k (observed) - C_b^n(0)_k * alpha_p,k (predicted) (Equation 29)
      // predicted_beta_p_from_state = C_b_n0_mat_global * current_alpha_p_from_state

      // alpha_p_padded để nhân với ma trận 3x3 C_b_n0_mat_global
      eekf_value current_alpha_p_padded[3] = {current_alpha_p_from_state[0], current_alpha_p_from_state[1], 0.0};
      eekf_value predicted_beta_p_3D[3];
      mat3x3_vec3x1_mul(C_b_n0_mat_global, current_alpha_p_padded, predicted_beta_p_3D);

      // Vector đo lường thực tế (z) cho EKF (chỉ lấy X,Y)
      *EEKF_MAT_EL(z, 0, 0) = beta_p_from_gps_actual[0] - predicted_beta_p_3D[0];
      *EEKF_MAT_EL(z, 1, 0) = beta_p_from_gps_actual[1] - predicted_beta_p_3D[1];

      // Cập nhật ma trận R (nhiễu đo lường) - giờ là 2x2
      // Bạn cần xác định giá trị noise này, có thể phụ thuộc vào HDOP GPS.
      // Ví dụ: update_R_matrix(g_hdop); // Bạn cần tự viết hàm này
      // Nếu không, R sẽ giữ giá trị khởi tạo.
      // update_R_matrix(g_hdop); // Gọi hàm này nếu bạn có

      double mahalanobis_squared = 0.0;
      bool is_gps_outlier = check_mahalanobis_distance(&ctx, &z, &R, &mahalanobis_squared); // z giờ là innovation

      // Tạm thời bỏ ghi đè cho mục đích gỡ lỗi, nhưng hãy nhớ bỏ chúng trong sản phẩm cuối cùng
      // is_gps_outlier=false;
      // attempt_gps_update=true;

      if (!is_gps_outlier) // Chỉ cập nhật nếu không phải ngoại lệ
      {
        eekf_correct(&ctx, &z, &R);
        g_ekf_operation_status = EKF_STATUS_PREDICT_UPDATE_GPS;
      }
      else
      {
        g_ekf_operation_status = EKF_STATUS_GPS_REJECTED;
      }
    }
  }
  // Vị trí Loci (alpha_p)
  ekf_ap_x = *EEKF_MAT_EL(*ctx.x, 0, 0);
  ekf_ap_y = *EEKF_MAT_EL(*ctx.x, 1, 0);

  // Vận tốc Loci (alpha_v)
  ekf_av_x = *EEKF_MAT_EL(*ctx.x, 2, 0);
  ekf_av_y = *EEKF_MAT_EL(*ctx.x, 3, 0);

  // Yaw thái độ
  ekf_yaw_rad = *EEKF_MAT_EL(*ctx.x, 4, 0);

  // Bias gia tốc kế
  ekf_ba_x = *EEKF_MAT_EL(*ctx.x, 5, 0);
  ekf_ba_y = *EEKF_MAT_EL(*ctx.x, 6, 0);

  // Bias con quay hồi chuyển Z
  ekf_bg_z = *EEKF_MAT_EL(*ctx.x, 7, 0);

  return true;
}

/**
 * @brief Kiểm tra khoảng cách Mahalanobis để phát hiện ngoại lệ GPS.
 * Khoảng cách Mahalanobis bình phương = e' * S^-1 * e, với e là vector đổi mới.
 *
 * @param p_ctx Con trỏ tới ngữ cảnh EKF.
 * @param p_z_actual Con trỏ tới ma trận vector đổi mới thực tế (innovation).
 * @param p_R_current Con trỏ tới ma trận hiệp phương sai nhiễu đo lường hiện tại.
 * @param out_mahalanobis_squared Con trỏ để lưu khoảng cách Mahalanobis bình phương.
 * @return true Nếu khoảng cách Mahalanobis vượt quá ngưỡng (là ngoại lệ), ngược lại false.
 */
bool check_mahalanobis_distance(eekf_context *p_ctx, eekf_mat const *p_z_actual, eekf_mat const *p_R_current, double *out_mahalanobis_squared)
{
  // Khai báo các ma trận cục bộ cần thiết
  EEKF_DECL_MAT_DYN(h_pred, NUM_MEASUREMENTS, 1);                // Cho h(x_pred) - Kích thước bằng p_z_actual
  EEKF_DECL_MAT_DYN(e_vec, NUM_MEASUREMENTS, 1);                 // Cho vector đổi mới (innovation)
  EEKF_DECL_MAT_DYN(H_mat, NUM_MEASUREMENTS, NUM_STATES);        // Cho Jacobian của h
  EEKF_DECL_MAT_DYN(H_P_mat, NUM_MEASUREMENTS, NUM_STATES);      // H * P_pred
  EEKF_DECL_MAT_DYN(H_trans, NUM_STATES, NUM_MEASUREMENTS);      // H_mat transpose
  EEKF_DECL_MAT_DYN(S_mat, NUM_MEASUREMENTS, NUM_MEASUREMENTS);  // Ma trận hiệp phương sai đổi mới (S = H*P*H' + R)
  EEKF_DECL_MAT_DYN(L_chol, NUM_MEASUREMENTS, NUM_MEASUREMENTS); // Cholesky factor của S
  EEKF_DECL_MAT_DYN(y_vec, NUM_MEASUREMENTS, 1);                 // Vector trung gian cho tính Mahalanobis

  *out_mahalanobis_squared = -1.0; // Giá trị mặc định nếu có lỗi

  // 1. Tính toán phép đo dự kiến (h_pred) và Jacobian Jh (H_mat) từ trạng thái dự đoán (p_ctx->x)
  eekf_return h_return_status = p_ctx->h(&h_pred, &H_mat, p_ctx->x, p_ctx->userData);
  if (h_return_status != eEekfReturnOk)
  {
    // Serial.println("Loi: Ham callback h that bai trong Mahalanobis check.");
    return true; // Coi như ngoại lệ nếu hàm h thất bại
  }

  // 2. Tính toán vector đổi mới (e = z_actual - h_pred)
  eekf_mat_sub(&e_vec, p_z_actual, &h_pred);

  // 3. (Đã có H_mat từ bước 1)

  // 4. Tính toán S = H * P_pred * H_transpose + R_current
  // Sử dụng p_ctx->P (hiệp phương sai dự đoán)
  eekf_mat_mul(&H_P_mat, &H_mat, p_ctx->P);  // H_P_mat = H * P
  eekf_mat_trs(&H_trans, &H_mat);            // H_trans = H'
  eekf_mat_mul(&S_mat, &H_P_mat, &H_trans);  // S_mat = H * P * H'
  eekf_mat_add(&S_mat, &S_mat, p_R_current); // S_mat = H * P * H' + R_current

  // 5. Tính toán Mahalanobis Distance Squared (D_M^2 = e' * S^-1 * e)
  if (NULL == eekf_mat_chol(&L_chol, &S_mat))
  { // Tính nhân tử Cholesky L của S
    // Serial.println("Loi: Phân rã Cholesky thất bại cho S trong Mahalanobis check (S không dương xác định).");
    return true; // Nếu S không dương xác định, coi là ngoại lệ
  }

  // Giải L * y = e (thế tiến)
  if (NULL == eekf_mat_fw_sub(&y_vec, &L_chol, &e_vec))
  {
    // Serial.println("Loi: Thế tiến thất bại trong Mahalanobis check.");
    return true; // Giải không thành công, coi là ngoại lệ
  }

  // D_M^2 = y_vec' * y_vec (tổng bình phương các phần tử của y_vec)
  double dm_sq = 0.0;
  for (uint8_t i = 0; i < y_vec.rows; i++)
  {
    dm_sq += (*EEKF_MAT_EL(y_vec, i, 0)) * (*EEKF_MAT_EL(y_vec, i, 0));
  }
  *out_mahalanobis_squared = dm_sq;

  // 6. Kiểm tra ngưỡng Mahalanobis
  if (dm_sq >= GPS_MAHALANOBIS_THRESHOLD)
  {
    return true; // Là ngoại lệ
  }

  return false; // Không phải ngoại lệ
}

void update_C_n_n0(eekf_value delta_t_sec, double current_lat, double current_lon, double current_height,
                   eekf_value current_vx, eekf_value current_vy, eekf_value current_vz)
{
  // Để thực sự chính xác, C_n(t)^n(0) cần được cập nhật liên tục từ omega_in^n
  // theo định nghĩa. Hiện tại, chúng ta sẽ đơn giản hóa nó.
  // Nếu bạn có C_n(t)^n(0) từ SINS quang, hãy sử dụng nó.
  // Nếu không, đây là một thách thức khác.

  // Đối với ví dụ này, chúng ta sẽ giả định một cách đơn giản rằng C_n(t)^n(0) không thay đổi quá nhiều
  // nếu điểm gốc không di chuyển nhiều, hoặc bạn cần một logic riêng để cập nhật nó.
  // Vì tính toán C_n(t)^n(0) cũng là một quá trình tích phân,
  // trong một hệ thống thực, bạn sẽ phải tích hợp omega_in^n để có được nó.
  // Đây là một chủ đề phức tạp khác.
  // Để đơn giản, có thể giả định C_n(t)^n(0) là ma trận đơn vị nếu frame n(0) = n(t) (chỉ đúng trong khoảng thời gian rất ngắn hoặc đứng yên).
  // Hoặc nó được tính toán từ các hàm bên ngoài dựa trên mô hình Trái đất.
  // Bài báo có thể đã giả định rằng C_n(t)^n(0) có thể được tính toán từ các giá trị navigation frame.

  // Cho mục đích minh họa, giả định C_n_n0_mat_elements được cập nhật đâu đó bên ngoài,
  // hoặc có thể nó là I nếu bạn đang làm việc trong một local-level frame.
  // C_n_n0_quat; // Bạn có thể cập nhật Quaternion này
  // Sau đó chuyển nó thành ma trận C_n_n0_mat_elements.
}

/**
 * @brief Chuyển đổi từ tọa độ Lat/Lon sang mét cục bộ ENU (X,Y - Đông, Bắc).
 * Giả định Trái Đất là mặt phẳng trong khu vực nhỏ.
 *
 * @param lat Vĩ độ (degrees).
 * @param lon Kinh độ (degrees).
 * @param L_origin Vĩ độ gốc (degrees).
 * @param Lon_origin Kinh độ gốc (degrees).
 * @param x_local Con trỏ để lưu tọa độ X cục bộ (meters).
 * @param y_local Con trỏ để lưu tọa độ Y cục bộ (meters).
 */
void geo_to_local_enu(double lat, double lon, double Lat_origin, double Lon_origin,
                      eekf_value *x_local, eekf_value *y_local)
{
  *x_local = (lon - Lon_origin) * METERS_PER_DEG_LON; // East:đông
  *y_local = (lat - Lat_origin) * METERS_PER_DEG_LAT; // North:bắc
}

/**
 * @brief Tính toán vector quan sát PIF (beta_p) từ lịch sử dữ liệu GPS.
 * Thực hiện tích lũy "sum of deltas" để giữ khả năng chống ngoại lệ.
 * Giả định Trái Đất là mặt phẳng và bỏ qua độ cao.
 *
 * @param num_points_in_history Số lượng điểm GPS hợp lệ trong lịch sử để tính toán.
 * @param beta_p_vec Con trỏ tới mảng 2 phần tử để lưu vector beta_p (X,Y).
 */
void calculate_beta_p(int num_points_in_history, eekf_value *beta_p_vec)
{
  beta_p_vec[0] = 0.0; // Khởi tạo tổng X
  beta_p_vec[1] = 0.0; // Khởi tạo tổng Y

  // Cần ít nhất 2 điểm để có delta (pk - pk-1)
  if (num_points_in_history < 2)
  {
    return;
  }

  // Tìm điểm đầu tiên trong circular buffer để bắt đầu vòng lặp
  // gps_history_idx là điểm tiếp theo sẽ ghi (index của dữ liệu MỚI NHẤT)
  // num_points_in_history là TỔNG SỐ điểm đã thu thập
  // Vòng lặp sẽ bắt đầu từ điểm thứ hai (offset = 1) so với điểm đầu tiên trong cửa sổ lịch sử.
  int first_point_in_window_idx = (gps_history_idx - num_points_in_history + MAX_GPS_HISTORY_POINTS) % MAX_GPS_HISTORY_POINTS;

  eekf_value current_r_n_x = 0.0; // Tích lũy tổng delta X
  eekf_value current_r_n_y = 0.0; // Tích lũy tổng delta Y

  // Lặp qua lịch sử GPS, tính delta giữa các điểm liên tiếp và tích lũy
  // Chúng ta bắt đầu từ điểm thứ hai trong cửa sổ để tính delta với điểm thứ nhất.
  for (int k_offset = 1; k_offset < num_points_in_history; ++k_offset)
  {
    // Chỉ số của điểm hiện tại (pk) trong lịch sử vòng tròn
    int idx_k = (first_point_in_window_idx + k_offset) % MAX_GPS_HISTORY_POINTS;
    // Chỉ số của điểm trước đó (pk-1) trong lịch sử vòng tròn
    int idx_k_minus_1 = (first_point_in_window_idx + k_offset - 1 + MAX_GPS_HISTORY_POINTS) % MAX_GPS_HISTORY_POINTS;

    // Lấy tọa độ cục bộ của hai điểm liên tiếp
    eekf_value pk_local_x = gps_history[idx_k].local_x;
    eekf_value pk_local_y = gps_history[idx_k].local_y;

    eekf_value pk_minus_1_local_x = gps_history[idx_k_minus_1].local_x;
    eekf_value pk_minus_1_local_y = gps_history[idx_k_minus_1].local_y;

    // Tính delta vị trí cục bộ (pk - pk-1)
    eekf_value delta_x_local = pk_local_x - pk_minus_1_local_x;
    eekf_value delta_y_local = pk_local_y - pk_minus_1_local_y;

    // Tích lũy các delta vào tổng
    current_r_n_x += delta_x_local;
    current_r_n_y += delta_y_local;
  }

  // gán tổng tích lũy vào beta_p_vec
  beta_p_vec[0] = current_r_n_x;
  beta_p_vec[1] = current_r_n_y;
}

/**
 * @brief Tính toán ma trận thái độ ban đầu (C_b^n(0)_k) bằng phương pháp OBA.
 *
 * @param current_alpha_p_vec_2D Vector alpha_p hiện tại từ trạng thái EKF (2D).
 * @param current_beta_p_vec_2D Vector beta_p hiện tại từ GPS (2D).
 * @param C_b_n0_mat Con trỏ tới mảng 9 phần tử để lưu ma trận 3x3 C_b^n(0)_k (thứ tự cột).
 */
void calculate_initial_attitude_OBA(const eekf_value *current_alpha_p_vec_2D,
                                    const eekf_value *current_beta_p_vec_2D,
                                    eekf_value *C_b_n0_mat)
{ // C_b_n0_mat is 3x3 output

  // Bước 1: Tính góc của vector alpha_p và beta_p
  eekf_value alpha_p_angle = atan2(current_alpha_p_vec_2D[1], current_alpha_p_vec_2D[0]);
  // Góc của vector beta_p
  eekf_value beta_p_angle = atan2(current_beta_p_vec_2D[1], current_beta_p_vec_2D[0]);
  // Serial.printf("gps_x: %.3f, gps_y: %.3f\n",
  //               gps_x, gps_y);
  // Serial.printf("Current alpha_p_vec_2D: (%.3f, %.3f)\n",
  //               current_alpha_p_vec_2D[0], current_alpha_p_vec_2D[1]);
  // Serial.printf("Current beta_p_vec_2D: (%.3f, %.3f)\n",
  //               current_beta_p_vec_2D[0], current_beta_p_vec_2D[1]);

  // Kiểm tra độ lớn của vector để tránh chia cho 0 hoặc lỗi khi vector là 0
  // Nếu vector có độ lớn rất nhỏ (thiết bị đứng yên), góc sẽ không đáng tin cậy.
  eekf_value alpha_p_magnitude_sq = current_alpha_p_vec_2D[0] * current_alpha_p_vec_2D[0] + current_alpha_p_vec_2D[1] * current_alpha_p_vec_2D[1];
  eekf_value beta_p_magnitude_sq = current_beta_p_vec_2D[0] * current_beta_p_vec_2D[0] + current_beta_p_vec_2D[1] * current_beta_p_vec_2D[1];

  // Serial.printf("alpha_p_angle: %.3f(deg), beta_p_angle: %.3f(deg), Difference: %.3f(deg)\n",
  //               degrees(alpha_p_angle), degrees(beta_p_angle), degrees(alpha_p_angle - beta_p_angle));
  // Ngưỡng nhỏ để xác định xem vector có gần bằng 0 hay không
  const eekf_value MIN_MAGNITUDE_SQ = 0.51 * 0.51; //

  eekf_value estimated_yaw0_rad;

  if (alpha_p_magnitude_sq < MIN_MAGNITUDE_SQ || beta_p_magnitude_sq < MIN_MAGNITUDE_SQ)
  {
    // Nếu một trong hai vector có độ lớn quá nhỏ (robot đứng yên hoặc vừa bắt đầu di chuyển),
    // thì phép tính góc không đáng tin cậy.
    // Giữ nguyên C_b_n0_mat_global (ma trận trước đó) hoặc đặt là Identity.
    // Để giữ tính ổn định, chúng ta sẽ không cập nhật C_b_n0_mat_global.
    // Điều này có nghĩa là yaw_0 sẽ không được hiệu chỉnh khi robot đứng yên.
    // Hoặc có thể đặt là 0 (hướng Bắc).
    estimated_yaw0_rad = g_yaw; // Giả định hướng Bắc ban đầu nếu không có chuyển động
  }
  else
  {
    // Bước 2: Tính góc Yaw ban đầu (yaw_0)
    // yaw_0 = góc của beta_p - góc của alpha_p
    // Điều này sẽ căn chỉnh alpha_p để chỉ theo beta_p.
    estimated_yaw0_rad = beta_p_angle - alpha_p_angle;

    // Chuẩn hóa yaw_0 về khoảng [-PI, PI)
    estimated_yaw0_rad = fmod(estimated_yaw0_rad + M_PI, 2.0 * M_PI);
    if (estimated_yaw0_rad < 0.0)
    {
      estimated_yaw0_rad += 2.0 * M_PI;
    }
    estimated_yaw0_rad -= M_PI;
  }
  // Serial.printf("Estimated yaw0 (rad): %.3f, Estimated yaw0 (deg): %.3f\n", estimated_yaw0_rad, degrees(estimated_yaw0_rad));

  // Bước 3: Xây dựng ma trận C_b^n(0) từ estimated_yaw0_rad
  // Ma trận 3x3 này sẽ chỉ chứa thông tin Yaw (và giả định Roll/Pitch = 0).
  eekf_value c_yaw0 = cos(estimated_yaw0_rad);
  eekf_value s_yaw0 = sin(estimated_yaw0_rad);

  memset(C_b_n0_mat, 0, sizeof(eekf_value) * 9); // Khởi tạo 0

  // Đây là ma trận xoay từ body frame ban đầu sang navigation frame (ENU)
  // C_b^n(0) = [ cY0 -sY0 0; sY0 cY0 0; 0 0 1 ]
  // (trong thứ tự cột của eekf_mat)
  C_b_n0_mat[0] = s_yaw0; // m00
  C_b_n0_mat[1] = c_yaw0; // m10
  C_b_n0_mat[2] = 0.0;    // m20

  C_b_n0_mat[3] = c_yaw0;  // m01
  C_b_n0_mat[4] = -s_yaw0; // m11
  C_b_n0_mat[5] = 0.0;     // m21

  C_b_n0_mat[6] = 0.0; // m02
  C_b_n0_mat[7] = 0.0; // m12
  C_b_n0_mat[8] = 1.0; // m22 (trục Z không thay đổi)
  /* // Bước 0: Khởi tạo S_k_mat nếu cần
  if (!S_k_initialized)
  {
    memset(S_k_mat.elements, 0, sizeof(eekf_value) * 16);
    S_k_initialized = true;
  }

  // Bước 1: Chuyển alpha_p và beta_p thành vector 4x1 với phần tử đầu tiên là 0 (Equation 34)
  // Alpha_p và Beta_p là 2D (chỉ X,Y). Phần Z sẽ là 0.
  eekf_value alpha_p_star[4] = {0.0, -current_alpha_p_vec_2D[0], -current_alpha_p_vec_2D[1], 0.0}; // Z = 0
  eekf_value beta_p_star[4] = {0.0, current_beta_p_vec_2D[0], current_beta_p_vec_2D[1], 0.0};      // Z = 0

  // Bước 2: Tính toán (beta* - alpha*) và tích lũy vào S_k
  // (beta_star - alpha_star) (vector 4x1)
  eekf_value diff_vector[4];
  for (int i = 0; i < 4; ++i)
  {
    diff_vector[i] = beta_p_star[i] - alpha_p_star[i];
  }

  // (beta_star - alpha_star)^T * (beta_star - alpha_star)
  // This is an outer product, resulting in a 4x4 matrix, which is then added to S_k_mat.
  for (int r = 0; r < 4; ++r)
  {
    for (int c = 0; c < 4; ++c)
    {
      *EEKF_MAT_EL(S_k_mat, r, c) += diff_vector[r] * diff_vector[c];
    }
  }

  // Bước 3: Tìm eigenvector của S_k_mat ứng với eigenvalue nhỏ nhất
  // Đây là phần khó nhất, yêu cầu thư viện hoặc triển khai thuật toán eigenvector (Jacobi, SVD).
  // Giả định có hàm `find_smallest_eigen_quat` (Bạn phải tự triển khai)
  // Nó sẽ lấy S_k_mat và trả về Quaternion của thái độ ban đầu.
  Quaternion q_b_n0_est; // Ước tính Quaternion của C_b^n(0)
  // find_smallest_eigen_quat(&S_k_mat, &q_b_n0_est);

  // **Để placeholder cho việc này, chúng ta sẽ giả định q_b_n0_est là quaternion đơn vị.**
  // **ĐIỀU NÀY LÀ KHÔNG ĐÚNG NẾU KHÔNG CÓ THUẬT TOÁN EIGENVECTOR THỰC TẾ.**
  q_b_n0_est.w = 1.0;
  q_b_n0_est.x = 0.0;
  q_b_n0_est.y = 0.0;
  q_b_n0_est.z = 0.0;

  // Bước 4: Chuyển Quaternion sang Ma trận xoay 3x3
  quat_to_rot_mat3x3(q_b_n0_est, C_b_n0_mat); */
}

void quat_to_rot_mat3x3(const Quaternion &q, eekf_value *mat_elements)
{
  // Ensure the quaternion is normalized (though typically handled by normalize() method)
  // q.normalize(); // Call if you're not sure it's normalized

  // Quaternion components
  eekf_value q0 = q.w;
  eekf_value q1 = q.x;
  eekf_value q2 = q.y;
  eekf_value q3 = q.z;

  // Pre-calculate squares to optimize
  eekf_value q0q0 = q0 * q0;
  eekf_value q1q1 = q1 * q1;
  eekf_value q2q2 = q2 * q2;
  eekf_value q3q3 = q3 * q3;

  // Pre-calculate products
  eekf_value q0q1 = q0 * q1;
  eekf_value q0q2 = q0 * q2;
  eekf_value q0q3 = q0 * q3;
  eekf_value q1q2 = q1 * q2;
  eekf_value q1q3 = q1 * q3;
  eekf_value q2q3 = q2 * q3;

  // Populate the 3x3 rotation matrix elements in column-major order
  // Column 0 (X-axis of body frame in navigation frame)
  mat_elements[0] = q0q0 + q1q1 - q2q2 - q3q3; // m00
  mat_elements[1] = 2 * (q1q2 + q0q3);         // m10
  mat_elements[2] = 2 * (q1q3 - q0q2);         // m20

  // Column 1 (Y-axis of body frame in navigation frame)
  mat_elements[3] = 2 * (q1q2 - q0q3);         // m01
  mat_elements[4] = q0q0 - q1q1 + q2q2 - q3q3; // m11
  mat_elements[5] = 2 * (q2q3 + q0q1);         // m21

  // Column 2 (Z-axis of body frame in navigation frame)
  mat_elements[6] = 2 * (q1q3 + q0q2);         // m02
  mat_elements[7] = 2 * (q2q3 - q0q1);         // m12
  mat_elements[8] = q0q0 - q1q1 - q2q2 + q3q3; // m22
}

void mat3x3_vec3x1_mul(const eekf_value *C_mat_elements, const eekf_value *vec, eekf_value *result)
{
  // result[0] = C_mat_elements[0*3+0] * vec[0] + C_mat_elements[0*3+1] * vec[1] + C_mat_elements[0*3+2] * vec[2]; (Incorrect for column-major)
  // Correct for column-major access: m[row + col*rows]
  // C_mat_elements[row + col*3] for 3x3 matrix

  // Result Row 0 (X component)
  result[0] = C_mat_elements[0] * vec[0] + C_mat_elements[3] * vec[1] + C_mat_elements[6] * vec[2];

  // Result Row 1 (Y component)
  result[1] = C_mat_elements[1] * vec[0] + C_mat_elements[4] * vec[1] + C_mat_elements[7] * vec[2];

  // Result Row 2 (Z component)
  result[2] = C_mat_elements[2] * vec[0] + C_mat_elements[5] * vec[1] + C_mat_elements[8] * vec[2];
}

Quaternion angular_rate_to_quat_rate(const Quaternion &q, eekf_value wx, eekf_value wy, eekf_value wz)
{
  // Create a quaternion from the angular rates (vector part only, scalar part is 0)
  Quaternion omega_quat(0.0, wx, wy, wz);

  // Calculate q_dot = 0.5 * q * omega_quat
  Quaternion q_dot_unscaled = q * omega_quat;

  // Scale by 0.5
  return Quaternion(
      0.5 * q_dot_unscaled.w,
      0.5 * q_dot_unscaled.x,
      0.5 * q_dot_unscaled.y,
      0.5 * q_dot_unscaled.z);
}

void create_skew_symmetric_mat(eekf_value v_x, eekf_value v_y, eekf_value v_z, eekf_value *skew_mat_elements)
{
  skew_mat_elements[0] = 0.0;
  skew_mat_elements[3] = -v_z;
  skew_mat_elements[6] = v_y; // Column 0
  skew_mat_elements[1] = v_z;
  skew_mat_elements[4] = 0.0;
  skew_mat_elements[7] = -v_x; // Column 1
  skew_mat_elements[2] = -v_y;
  skew_mat_elements[5] = v_x;
  skew_mat_elements[8] = 0.0; // Column 2
}

// Function to multiply 3x3 matrix by 3x3 matrix (column-major)
// Result = A * B
void mat3x3_mat3x3_mul(const eekf_value *A, const eekf_value *B, eekf_value *Result)
{
  Result[0] = A[0] * B[0] + A[3] * B[1] + A[6] * B[2];
  Result[3] = A[0] * B[3] + A[3] * B[4] + A[6] * B[5];
  Result[6] = A[0] * B[6] + A[3] * B[7] + A[6] * B[8];
  Result[1] = A[1] * B[0] + A[4] * B[1] + A[7] * B[2];
  Result[4] = A[1] * B[3] + A[4] * B[4] + A[7] * B[5];
  Result[7] = A[1] * B[6] + A[4] * B[7] + A[7] * B[8];
  Result[2] = A[2] * B[0] + A[5] * B[1] + A[8] * B[2];
  Result[5] = A[2] * B[3] + A[5] * B[4] + A[8] * B[5];
  Result[8] = A[2] * B[6] + A[5] * B[7] + A[8] * B[8];
}