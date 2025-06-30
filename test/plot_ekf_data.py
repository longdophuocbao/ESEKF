import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.widgets import TextBox
import matplotlib.animation as animation
import serial
import io
from datetime import datetime
import atexit

# --- Cấu hình ---
SERIAL_PORT = 'COM3'  # THAY THẾ BẰNG CỔNG COM CỦA ESP32 (ví dụ: 'COM3' trên Windows, '/dev/ttyUSB0' trên Linux)
BAUD_RATE = 115200
UPDATE_INTERVAL_SECONDS = 0.5   # Tần suất cập nhật đồ thị (giây)
MAX_DATA_POINTS = 500          # Tăng số điểm dữ liệu tối đa hiển thị trên đồ thị trượt để đường đi rõ hơn

OUTPUT_CSV_FILENAME = "ekf_data_log_2.csv"
CSV_COLUMNS = [
    "Timestamp_ms",
    "Raw_Accel_X", "Raw_Accel_Y", "Raw_Accel_Z",
    "Unbiased_Filt_Accel_X", "Unbiased_Filt_Accel_Y",
    "Raw_Omega_Z_degs", "Unbiased_Omega_Z_rads",
    "IMU_Roll_deg", "IMU_Pitch_deg", "IMU_Yaw_deg",
    "GPS_Lat", "GPS_Lon", "GPS_GroundSpeed_kmh", "GPS_Satellites", "GPS_HDOP",
    "GPS_Local_X_m", "GPS_Local_Y_m",
    "EKF_ap_X_m", "EKF_ap_Y_m", # Tên cột mới
    "EKF_av_X_mps", "EKF_av_Y_mps",
    "EKF_q_W", "EKF_q_X", "EKF_q_Y", "EKF_q_Z",
    "EKF_ba_X_mps2", "EKF_ba_Y_mps2",
    "EKF_bg_Z_rads",
    "EKF_Yaw_deg",
    "P_ap_X", "P_ap_Y", "P_av_X", "P_av_Y", "P_q_W", "P_ba_X", "P_ba_Y", "P_bg_Z",
    "R_delta_pos_X", "R_delta_pos_Y",
    "Q_av", "Q_q", "Q_ba", "Q_bg",
    "ZUPT_Actived", "EKF_Status", "Delta_T_s"
]

# Khởi tạo Serial
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"Đang kết nối tới cổng {SERIAL_PORT}...")
    if ser.is_open:
        print("Đang chờ header từ ESP32...")
        header_line = ser.readline().decode('utf-8', errors='ignore').strip()
        print(f"Header nhận được: {header_line}")
        if not header_line.startswith("Timestamp_ms"):
            print("CẢNH BÁO: Header nhận được có vẻ không đúng. Đảm bảo ESP32 đang gửi header CSV.")
        else:
            print("Header hợp lệ. Bắt đầu nhận dữ liệu...")
    else:
        print(f"Không thể mở cổng {SERIAL_PORT}.")
        exit()
except serial.SerialException as e:
    print(f"Lỗi khi mở cổng serial {SERIAL_PORT}: {e}")
    print("Vui lòng kiểm tra lại cổng COM và đảm bảo ESP32 đang kết nối.")
    exit()


# Khởi tạo DataFrame rỗng
data_df = pd.DataFrame(columns=CSV_COLUMNS)
full_data_df = pd.DataFrame(columns=CSV_COLUMNS)

# --- Thiết lập đồ thị ---
fig, axs = plt.subplots(4, 2, figsize=(15, 13))
plt.subplots_adjust(bottom=0.15, hspace=0.5, wspace=0.3) # Tăng khoảng cách giữa các subplot

# Đồ thị 1: Vị trí EKF và GPS Local Path (X vs Y) - ĐÂY ĐÃ LÀ ĐƯỜNG ĐI
line_ekf_xy, = axs[0, 0].plot([], [], 'r-', label='EKF Path (m)')
line_gps_xy, = axs[0, 0].plot([], [], 'b--', label='GPS Local Path (m)')
axs[0, 0].set_xlabel("X (m)")
axs[0, 0].set_ylabel("Y (m)")
axs[0, 0].set_title("EKF vs GPS Local Path")
axs[0, 0].legend()
axs[0, 0].grid(True)
axs[0, 0].axis('equal') # Giữ tỷ lệ trục bằng nhau để đường đi không bị méo

# Đồ thị 2: Góc Yaw (EKF vs IMU theo thời gian)
line_ekf_theta, = axs[0, 1].plot([], [], 'c-', label='EKF Yaw (deg)')
line_imu_yaw, = axs[0, 1].plot([], [], 'y-', label='IMU Yaw (deg)')
axs[0, 1].set_xlabel("Time (s)")
axs[0, 1].set_ylabel("Yaw (degrees)")
axs[0, 1].set_title("Yaw Angle")
axs[0, 1].legend()
axs[0, 1].grid(True)

# Đồ thị 3: Vận tốc EKF (VX, VY theo thời gian)
line_ekf_vx, = axs[1, 0].plot([], [], 'g-', label='EKF VX (m/s)')
line_ekf_vy, = axs[1, 0].plot([], [], 'm-', label='EKF VY (m/s)')
axs[1, 0].set_xlabel("Time (s)")
axs[1, 0].set_ylabel("Velocity (m/s)")
axs[1, 0].set_title("EKF Velocities")
axs[1, 0].legend()
axs[1, 0].grid(True)

# Đồ thị 4: Phương sai P (P_ap_X, P_ap_Y theo thời gian) - THAY ĐỔI TÊN CỘT
line_p_ap_x, = axs[1, 1].plot([], [], 'k-', label='P_ap_X (m^2)') # Thay thế P_x bằng P_ap_X
line_p_ap_y, = axs[1, 1].plot([], [], 'lime', label='P_ap_Y (m^2)') # Thay thế P_y bằng P_ap_Y
axs[1, 1].set_xlabel("Time (s)")
axs[1, 1].set_ylabel("Position Variance (m^2)")
axs[1, 1].set_title("EKF Position Variances (P)")
axs[1, 1].legend()
axs[1, 1].grid(True)

# Đồ thị 5: Accel Biases (b_ax, b_ay)
line_ba_x, = axs[2, 0].plot([], [], 'darkred', label='EKF b_ax (m/s^2)')
line_ba_y, = axs[2, 0].plot([], [], 'darkblue', label='EKF b_ay (m/s^2)')
axs[2, 0].set_xlabel("Time (s)")
axs[2, 0].set_ylabel("Bias (m/s^2)")
axs[2, 0].set_title("EKF Accel Biases")
axs[2, 0].legend()
axs[2, 0].grid(True)

# Đồ thị 6: Gyro Bias (b_gz) và Delta_T_s
line_bg_z, = axs[2, 1].plot([], [], 'darkgreen', label='EKF b_gz (rad/s)')
line_delta_t, = axs[2, 1].plot([], [], 'purple', linestyle='--', label='Delta_T (s)') # Đổi lại đơn vị Delta_T cho phù hợp với CSV
axs[2, 1].set_xlabel("Time (s)")
axs[2, 1].set_ylabel("Bias (rad/s) / Delta T (s)")
axs[2, 1].set_title("EKF Gyro Bias & Delta T")
axs[2, 1].legend(loc='upper left')
axs[2, 1].grid(True)

# Đồ thị 7: Gia tốc (Raw vs Filtered/Unbiased)
line_raw_ax, = axs[3, 0].plot([], [], 'r--', alpha=0.7, label='Raw Accel X (m/s^2)')
line_raw_ay, = axs[3, 0].plot([], [], 'b--', alpha=0.7, label='Raw Accel Y (m/s^2)')
line_filt_ax, = axs[3, 0].plot([], [], 'r-', label='Filt/Unbias Accel X (m/s^2)')
line_filt_ay, = axs[3, 0].plot([], [], 'b-', label='Filt/Unbias Accel Y (m/s^2)')
axs[3, 0].set_xlabel("Time (s)")
axs[3, 0].set_ylabel("Acceleration (m/s^2)")
axs[3, 0].set_title("Accelerations (Raw vs. Processed)")
axs[3, 0].legend(loc='upper left')
axs[3, 0].grid(True)

# Đồ thị 8: Tốc độ góc (Raw vs Unbiased)
line_raw_wz, = axs[3, 1].plot([], [], 'g--', alpha=0.7, label='Raw Omega Z (deg/s)')
line_unbias_wz, = axs[3, 1].plot([], [], 'g-', label='Unbiased Omega Z (rad/s)')
axs[3, 1].set_xlabel("Time (s)")
axs[3, 1].set_ylabel("Angular Velocity")
axs[3, 1].set_title("Angular Velocity Z (Raw vs. Processed)")
axs[3, 1].legend(loc='upper left')
axs[3, 1].grid(True)

# --- Hàm gửi lệnh qua Serial ---
def send_serial_command(text):
    if ser.is_open:
        try:
            command_to_send = text + '\n'
            ser.write(command_to_send.encode('utf-8'))
            print(f"Đã gửi lệnh: {text}")
        except Exception as e:
            print(f"Lỗi khi gửi lệnh qua serial: {e}")
    else:
        print("Cổng serial chưa mở. Không thể gửi lệnh.")

# --- Tạo TextBox để nhập lệnh ---
ax_textbox = fig.add_axes([0.1, 0.03, 0.8, 0.05])
text_box_initial_text = "0.1,0.01,0.5,0.6" # Các giá trị ví dụ cho sigma_accel_noise, sigma_gyro_noise, sigma_accel_bias_walk, sigma_gyro_bias_walk
text_box = TextBox(ax_textbox, "Sent:", initial=text_box_initial_text)
text_box.on_submit(send_serial_command)

# --- Hàm lưu dữ liệu vào CSV khi thoát ---
def save_data_to_csv():
    if not full_data_df.empty: # Lưu toàn bộ dữ liệu
        try:
            full_data_df.to_csv(OUTPUT_CSV_FILENAME, index=False)
            print(f"\nĐã lưu dữ liệu vào file: {OUTPUT_CSV_FILENAME}")
        except Exception as e:
            print(f"Lỗi khi lưu file CSV: {e}")
atexit.register(save_data_to_csv)

def update_data():
    """Đọc dữ liệu mới từ serial và thêm vào DataFrame."""
    global data_df, full_data_df
    if ser.is_open and ser.in_waiting > 0:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line and not line.startswith("Timestamp_ms"):
                values = line.split(',')
                if len(values) == len(CSV_COLUMNS):
                    new_row = pd.Series(values, index=CSV_COLUMNS)

                    # Thêm vào cả hai DataFrames
                    # Chuyển đổi Series thành DataFrame trước khi concat để tránh cảnh báo
                    data_df = pd.concat([data_df, new_row.to_frame().T], ignore_index=True)
                    full_data_df = pd.concat([full_data_df, new_row.to_frame().T], ignore_index=True)

                    # Chuyển đổi kiểu dữ liệu cho các cột số
                    for col in CSV_COLUMNS: # Duyệt qua CSV_COLUMNS thay vì data_df.columns
                        try:
                            # Tránh chuyển đổi các cột không phải số (ví dụ ZUPT_Actived, EKF_Status)
                            if col not in ["ZUPT_Actived", "EKF_Status"]:
                                data_df[col] = pd.to_numeric(data_df[col], errors='coerce')
                                full_data_df[col] = pd.to_numeric(full_data_df[col], errors='coerce')
                        except Exception:
                            # Bỏ qua lỗi nếu cột không phải số
                            pass

                    # Giới hạn số lượng điểm dữ liệu cho data_df (cho đồ thị trượt)
                    if len(data_df) > MAX_DATA_POINTS:
                        data_df = data_df.iloc[-MAX_DATA_POINTS:].reset_index(drop=True) # reset index sau iloc
                else:
                    print(f"CẢNH BÁO: Số lượng cột không khớp. Mong đợi {len(CSV_COLUMNS)}, nhận được {len(values)}. Dòng: {line}")
        except Exception as e:
            print(f"Lỗi khi đọc hoặc xử lý dòng từ serial: {e}")

def animate(i):
    """Hàm được gọi bởi FuncAnimation để cập nhật đồ thị."""
    update_data()

    if not data_df.empty:
        # Lấy dữ liệu thời gian (giây) từ Timestamp_ms
        time_s = data_df["Timestamp_ms"].astype(float) / 1000.0
        # Nếu data_df bị cắt bằng iloc, time_s.iloc[0] có thể không phải là 0
        # time_s_relative = time_s - time_s.iloc[0] # Thời gian tương đối so với điểm đầu tiên trong khung hình hiện tại
        time_s_relative = time_s - time_s.min() # Thời gian tương đối so với điểm nhỏ nhất trong khung hình hiện tại


        # Cập nhật đồ thị 1: EKF Path (EKF_ap_X_m, EKF_ap_Y_m)
        line_ekf_xy.set_data(data_df["EKF_ap_X_m"], data_df["EKF_ap_Y_m"])
        # Cập nhật GPS Local Path (GPS_Local_X_m, GPS_Local_Y_m)
        line_gps_xy.set_data(data_df["GPS_Local_X_m"], data_df["GPS_Local_Y_m"])
        axs[0, 0].relim()
        axs[0, 0].autoscale_view()
        axs[0, 0].axis('equal') # Giữ tỷ lệ trục bằng nhau

        # Cập nhật đồ thị 2: Yaw Angle (EKF_Yaw_deg, IMU_Yaw_deg)
        line_ekf_theta.set_data(time_s_relative, data_df["EKF_Yaw_deg"])
        line_imu_yaw.set_data(time_s_relative, data_df["IMU_Yaw_deg"])
        axs[0, 1].relim()
        axs[0, 1].autoscale_view()

        # Cập nhật đồ thị 3: EKF Velocities (EKF_av_X_mps, EKF_av_Y_mps)
        line_ekf_vx.set_data(time_s_relative, data_df["EKF_av_X_mps"])
        line_ekf_vy.set_data(time_s_relative, data_df["EKF_av_Y_mps"])
        axs[1, 0].relim()
        axs[1, 0].autoscale_view()

        # Cập nhật đồ thị 4: Position Variances (P_ap_X, P_ap_Y)
        line_p_ap_x.set_data(time_s_relative, data_df["P_ap_X"])
        line_p_ap_y.set_data(time_s_relative, data_df["P_ap_Y"])
        axs[1, 1].relim()
        axs[1, 1].autoscale_view()
        
        # Cập nhật đồ thị 5: Accel Biases (EKF_ba_X_mps2, EKF_ba_Y_mps2)
        line_ba_x.set_data(time_s_relative, data_df["EKF_ba_X_mps2"])
        line_ba_y.set_data(time_s_relative, data_df["EKF_ba_Y_mps2"])
        axs[2, 0].relim()
        axs[2, 0].autoscale_view()

        # Cập nhật đồ thị 6: Gyro Bias (EKF_bg_Z_rads) & Delta_T (Delta_T_s)
        line_bg_z.set_data(time_s_relative, data_df["EKF_bg_Z_rads"])
        line_delta_t.set_data(time_s_relative, data_df["Delta_T_s"])
        axs[2, 1].relim()
        axs[2, 1].autoscale_view()

        # Cập nhật đồ thị 7: Accelerations
        line_raw_ax.set_data(time_s_relative, data_df["Raw_Accel_X"])
        line_raw_ay.set_data(time_s_relative, data_df["Raw_Accel_Y"])
        line_filt_ax.set_data(time_s_relative, data_df["Unbiased_Filt_Accel_X"])
        line_filt_ay.set_data(time_s_relative, data_df["Unbiased_Filt_Accel_Y"])
        axs[3, 0].relim()
        axs[3, 0].autoscale_view()

        # Cập nhật đồ thị 8: Angular Velocities
        line_raw_wz.set_data(time_s_relative, data_df["Raw_Omega_Z_degs"])
        line_unbias_wz.set_data(time_s_relative, data_df["Unbiased_Omega_Z_rads"])
        axs[3, 1].relim()
        axs[3, 1].autoscale_view()

    return (line_ekf_xy, line_gps_xy, line_ekf_vx, line_ekf_vy,
            line_ekf_theta, line_imu_yaw, line_p_ap_x, line_p_ap_y, line_delta_t,
            line_raw_ax, line_raw_ay, line_filt_ax, line_filt_ay,
            line_raw_wz, line_unbias_wz,
            line_ba_x, line_ba_y, line_bg_z) # Thêm các line mới vào giá trị trả về

# Tạo animation
ani = animation.FuncAnimation(fig, animate, interval=int(UPDATE_INTERVAL_SECONDS * 1000), blit=False, cache_frame_data=False)

try:
    plt.show()
except KeyboardInterrupt:
    print("\nĐã nhận KeyboardInterrupt, đang thoát và lưu dữ liệu...")
finally:
    if ser.is_open:
        ser.close()
        print(f"Đã đóng cổng {SERIAL_PORT}.")
    # save_data_to_csv()