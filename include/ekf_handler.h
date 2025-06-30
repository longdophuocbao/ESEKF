#ifndef EKF_HANDLER_H
#define EKF_HANDLER_H

#include "globals.h" // Để truy cập các biến toàn cục, hằng số và kiểu ekf_t
#include "serial_communication.h"
#include "sensor_processing.h"
#include <math.h>

// --- Global variables for OBA calculation ---
// S_k matrix (Equation 33)

// bool S_k_initialized; // Flag to initialize S_k_mat once

// --- Hàm hỗ trợ cần thêm (prototype) ---
// Function declarations for quaternion helpers
void quat_to_rot_mat3x3(const Quaternion &q, eekf_value *mat_elements);
void mat3x3_vec3x1_mul(const eekf_value *C_mat_elements, const eekf_value *vec, eekf_value *result); // Sử dụng eekf_value*
Quaternion angular_rate_to_quat_rate(const Quaternion &q, eekf_value wx, eekf_value wy, eekf_value wz);

// Các hàm hỗ trợ cho PIF/OBA
void geo_to_local_enu(double lat, double lon, double Lat_origin, double Lon_origin, eekf_value *x_local, eekf_value *y_local);
void calculate_beta_p(int num_points_in_history, eekf_value *beta_p_vec);
void calculate_initial_attitude_OBA(const eekf_value *current_alpha_p_vec_2D,
                                    const eekf_value *current_beta_p_vec_2D,
                                    eekf_value *C_b_n0_mat);
void create_skew_symmetric_mat(eekf_value v_x, eekf_value v_y, eekf_value v_z, eekf_value *skew_mat_elements);
void mat3x3_mat3x3_mul(const eekf_value *A, const eekf_value *B, eekf_value *Result);
void initialize_ekf_parameters();
eekf_return transition(eekf_mat *xp, eekf_mat *Jf, eekf_mat const *x,
                       eekf_mat const *u, void *userData);
eekf_return measurement(eekf_mat *zp, eekf_mat *Jh, eekf_mat const *x,
                        void *userData);
bool check_mahalanobis_distance(eekf_context *p_ctx, eekf_mat const *p_z_actual, eekf_mat const *p_R_current, double *out_mahalanobis_squared);
bool ExtendedKalmanFilter(bool *zupt_flag_out, bool p_is_gps_valid);

#endif // EKF_HANDLER_H
