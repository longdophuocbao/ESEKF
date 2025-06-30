#ifndef SERIAL_COMMUNICATION_H
#define SERIAL_COMMUNICATION_H

#include "globals.h" // Để truy cập các biến toàn cục và hằng số

// Khai báo các hàm
void print_csv_header();
void print_csv_data(bool zupt_activated_flag, double current_delta_t_us);
void handleSerialInput();
void print_csv_header_2();
void print_csv_data_2();
void printDebug1();
void printDebug2();

#endif // SERIAL_COMMUNICATION_H
