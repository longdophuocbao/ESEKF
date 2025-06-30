import pandas as pd
import io

# --- Cấu hình ---
INPUT_DATA_FILE = 'data_raw_27062025.csv' # Tên file dữ liệu thô đầu vào của bạn
OUTPUT_C_HEADER_FILE = 'imu_gps_data_27062025.h'   # Tên file header C++ đầu ra

# Tên các cột CSV (phải khớp với print_csv_header trong C++)
CSV_COLUMNS = [
    "Timestamp_ms",
    "Raw_Accel_X", "Raw_Accel_Y", "Raw_Accel_Z",
    "Unbiased_Filt_Accel_X", "Unbiased_Filt_Accel_Y",
    "Raw_Omega_Z_degs", "Unbiased_Omega_Z_rads",
    "IMU_Roll_deg", "IMU_Pitch_deg", "IMU_Yaw_deg",
    "GPS_Lat", "GPS_Lon", "GPS_GroundSpeed_kmh", "GPS_Satellites", "GPS_HDOP",
    "GPS_Local_X_m", "GPS_Local_Y_m",
    "R_delta_pos_X", "R_delta_pos_Y",
    "Q_av", "Q_q", "Q_ba", "Q_bg",
    "ZUPT_Actived", "EKF_Status", "Delta_T_s"
]

# Các cột dữ liệu thô mà bạn muốn trích xuất
DATA_COLUMNS_TO_EXTRACT = [
    "Timestamp_ms",
    "Raw_Accel_X", "Raw_Accel_Y", "Raw_Accel_Z",
    "Raw_Omega_Z_degs",
    "IMU_Yaw_deg", "IMU_Roll_deg", "IMU_Pitch_deg", # Góc từ IMU
    "GPS_Lat", "GPS_Lon", "GPS_GroundSpeed_kmh", "GPS_Satellites", "GPS_HDOP"
]

DATA_COLUMNS_TO_EXTRACT_2 = [
    "Count","Timestamp_s",
    "Raw_Accel_X", "Raw_Accel_Y",
    "Raw_Omega_Z_degs",
    "IMU_Yaw_deg", 
    "GPS_Lon","GPS_Lat", "GPS_GroundSpeed_kmh", "GPS_Satellites", "GPS_HDOP"
]

def generate_c_header(input_file, output_header_file, columns_to_extract, start_line=0, max_lines_to_read=None):
    """
    Tạo một file header C/C++ từ dữ liệu CSV, cho phép chỉ định dòng bắt đầu
    và số lượng dòng tối đa cần đọc.

    Args:
        input_file (str): Đường dẫn đến file CSV đầu vào.
        output_header_file (str): Đường dẫn đến file header C/C++ đầu ra.
        columns_to_extract (list): Danh sách các tên cột cần trích xuất.
        start_line (int): Dòng bắt đầu đọc dữ liệu (dựa trên 0).
                            Mặc định là 0 (đầu file).
        max_lines_to_read (int, optional): Số lượng dòng tối đa cần đọc sau dòng bắt đầu.
                                           Nếu là None, đọc tất cả các dòng còn lại.
    """
    try:
        # Đọc dữ liệu từ file CSV
        with open(input_file, 'r', encoding='utf-8', errors='ignore') as f:
            lines = f.readlines()

        # Tìm dòng header thực tế
        header_line_index = -1
        for i, line in enumerate(lines):
            if line.strip().lower().startswith("count"): # Đảm bảo kiểm tra không phân biệt chữ hoa chữ thường và bỏ khoảng trắng thừa
                header_line_index = i
                break

        if header_line_index == -1:
            print(f"Lỗi: Không tìm thấy dòng header bắt đầu bằng 'Count' trong {input_file}.")
            return

        # Xác định dòng dữ liệu bắt đầu sau header và sau start_line
        data_start_index = header_line_index + 1 + start_line

        if data_start_index >= len(lines):
            print(f"Lỗi: Dòng bắt đầu đọc ({start_line}) vượt quá số lượng dòng dữ liệu có sẵn sau header.")
            return

        # Tạo một StringIO object từ các dòng dữ liệu, bắt đầu từ header
        # Chúng ta cần giữ lại header để pandas có thể phân tích đúng
        # và sau đó cắt các dòng dữ liệu để chỉ bao gồm phần mong muốn.
        
        # Lấy header line
        header_line = lines[header_line_index]
        
        # Lấy các dòng dữ liệu từ data_start_index
        data_lines_to_process = lines[data_start_index:]

        # Giới hạn số lượng dòng nếu max_lines_to_read được chỉ định
        if max_lines_to_read is not None:
            data_lines_to_process = data_lines_to_process[:max_lines_to_read]
        
        # Kết hợp header và các dòng dữ liệu đã lọc vào StringIO
        data_io = io.StringIO(header_line + "".join(data_lines_to_process))
        
        # Sử dụng pandas để đọc dữ liệu. header=0 chỉ định rằng hàng đầu tiên của data_io là header.
        # names=DATA_COLUMNS_TO_EXTRACT_2 được sử dụng nếu bạn muốn gán lại tên cột.
        # nrows không còn cần thiết ở đây vì chúng ta đã cắt các dòng trước khi đưa vào StringIO.
        df = pd.read_csv(data_io, header=0, names=DATA_COLUMNS_TO_EXTRACT_2)
        
        # Lọc các cột cần thiết
        df_extracted = df[columns_to_extract].copy()

        # Chuyển đổi sang kiểu số (nếu chưa) và xử lý NaN
        for col in df_extracted.columns:
            df_extracted[col] = pd.to_numeric(df_extracted[col], errors='coerce')
        df_extracted = df_extracted.fillna(0.0) # Thay thế NaN bằng 0.0

        num_records = len(df_extracted)

        # Mở file header C++ để ghi
        with open(output_header_file, 'w') as f_out:
            f_out.write("#ifndef IMU_GPS_DATA_H\n")
            f_out.write("#define IMU_GPS_DATA_H\n\n")
            f_out.write("#include <stdint.h>\n\n")
            
            f_out.write(f"const int NUM_SAMPLE_RECORDS = {num_records};\n\n")

            # Ghi các mảng dữ liệu
            for col_name in columns_to_extract:
                clean_col_name = col_name.replace('.', '_').replace('-', '_').replace(' ', '_') # Xóa ký tự không hợp lệ cho tên biến C++ và thay thế khoảng trắng
                f_out.write(f"const double {clean_col_name}[] = {{\n    ")
                
                # Định dạng các số thập phân cho C++ (sử dụng .10f hoặc tương tự)
                f_out.write(", ".join([f"{val:.10f}" for val in df_extracted[col_name]]))
                f_out.write("\n};\n\n")

            f_out.write("#endif // IMU_GPS_DATA_H\n")

        print(f"Đã tạo thành công file header: {output_header_file}")
        print(f"Tổng số bản ghi: {num_records}")

    except FileNotFoundError:
        print(f"Lỗi: Không tìm thấy file dữ liệu đầu vào '{input_file}'.")
    except Exception as e:
        print(f"Đã xảy ra lỗi: {e}")

if __name__ == "__main__":
    # Ví dụ: đọc 12000 dòng bắt đầu từ dòng 2000 (sau header)
    # Nếu dòng header là dòng 5, và bạn muốn bắt đầu đọc dữ liệu từ dòng 2000 sau đó,
    # thì `start_line` sẽ là 2000.
    # Dòng 0: Dòng đầu tiên của file CSV
    # Dòng header: Dòng mà 'Count' xuất hiện (ví dụ: dòng thứ 5)
    # Dữ liệu bắt đầu từ: Dòng header + 1
    # Nếu bạn muốn bỏ qua 2000 dòng dữ liệu đầu tiên (sau header),
    # bạn sẽ đặt start_line = 2000.
    generate_c_header(INPUT_DATA_FILE, OUTPUT_C_HEADER_FILE, DATA_COLUMNS_TO_EXTRACT_2, start_line=2000, max_lines_to_read=12000)

    # Để đọc tất cả các dòng như trước, bỏ qua tham số start_line và max_lines_to_read
    # generate_c_header(INPUT_DATA_FILE, OUTPUT_C_HEADER_FILE, DATA_COLUMNS_TO_EXTRACT_2)
