Bộ Lọc Kalman Mở Rộng Ước Lượng Trạng Thái (ESEKF)
* **Tích hợp IMU/GPS bằng EKF:** Kết hợp dữ liệu gia tốc, tốc độ góc từ IMU và vị trí từ GPS.
* **Mô hình EKF 8 Trạng thái:**
    * `alpha_p_x`, `alpha_p_y`: Vector Quỹ đạo Vị trí (Position Loci Vector) - ước tính đường đi tương đối.
    * `alpha_v_x`, `alpha_v_y`: Vector Tham chiếu Vận tốc (Velocity Reference Vector) - ước tính vận tốc.
    * `yaw`: Góc Yaw (Hướng) của robot.
    * `b_ax`, `b_ay`: Bias của Gia tốc kế (Accelerometer Biases).
    * `b_gz`: Bias của Con quay hồi chuyển trục Z (Gyroscope Bias Z).
* **4 Phép đo EKF:**
    * Sai số vị trí X, Y (từ phương pháp PIF).
    * Sai số Yaw (từ GPS Heading).
    * Sai số Tốc độ (từ GPS Speed).
  **Phương pháp PIF (Position Integration Formula):** Sử dụng tích hợp đầy đủ các thay đổi vị trí từ GPS để tạo vector quan sát, giúp làm mượt nhiễu và chống ngoại lệ.
* Bo mạch ESP32 (ví dụ: ESP32 Dev Module)
* Cảm biến IMU WTGAHRS3/485
