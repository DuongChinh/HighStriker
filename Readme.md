PROJECT HỆ NHÚNG
Mô tả đề tài: Xây dựng phần cứng và phần mềm mô phỏng trò chơi High Striker. 

GIỚI THIỆU
Đề bài: Xây dựng phần cứng và phần mềm mô phỏng trò chơi High Striker.

Sản phẩm:
- Các tính năng chính:
1. Hệ thống cho phép đo lực tối đa mà người dùng tác động lên cảm biến
•	Đo lực tác động từ người chơi qua load cell (thường tích hợp với mạch HX711).
•	Ghi nhận lực đập lớn nhất trong mỗi lượt chơi.
•	Đảm bảo tốc độ lấy mẫu đủ nhanh để không bỏ lỡ đỉnh lực tác động (peak force).
2. Hiển thị giá trị lực đo được thông qua cột LED
•	Hiển thị mức lực tương ứng với chiều cao LED sáng.
•	Càng đập mạnh, số lượng LED sáng càng cao (mô phỏng cột dọc trong trò chơi High Striker).
3. Hiển thị trên màn hình TFT LCD.
•	Hiển thị số liệu lực hiện tại.
•	Hiển thị giá trị kỷ lục cao nhất.
•	Giao diện bắt mắt, màu sắc sinh động.
4. Ghi nhớ và hiển thị kỷ lục cao nhất
•	Lưu giữ lực đập mạnh nhất từng đo trong nhiều lần chơi.
•	So sánh kết quả mới với kỷ lục để xác định "New Record!".

TÁC GIẢ
•	Tên nhóm: 3 chang linh ngu lam
•	Thành viên trong nhóm
| STT | Họ tên           | MSSV     | Công việc 							 				   |
|-----|------------------|----------|----------------------------------------------------------|
| 1   | Dương Xuân Chính | 20215534 |Phần cứng & Tích hợp cảm biến           				   |
| 2   | Lê Việt Quang    | 20215630 |Logic xử lý, UART debug								   |
| 3   | Lê Văn Sơn       | 20210752 |Hiển thị & Giao diện người dùng qua Hercules, viết báo cáo|

MÔI TRƯỜNG HOẠT ĐỘNG
•	STM32F429I-DISC1 (STM32F429 Discovery Kit)
o	MCU: STM32F429ZIT6 (ARM Cortex-M4, 180 MHz, 2 MB Flash, 256 KB RAM)
o	Tích hợp:
o	Màn hình TFT LCD 2.4” 320x240
o	ST-LINK/V2-1 debugger
o	Accelerometer, MEMS microphone (không sử dụng trong dự án này)
o	GPIO headers, USB OTG
•	Các module / kit được sử dụng trong dự án
o	Load Cell 20kg + HX711 Module
o	6 đèn LED 
o	STM32F429I-DISC1 TFT LCD
o	Nguồn 5V
o	breadboard
SƠ ĐỒ 
Cho biết cách nối dây, kết nối giữa các linh kiện 
| STM32F429         | Module Ngoại Vi                   | Chức Năng                                  |
|-------------------|-----------------------------------|--------------------------------------------|
| `PA11`            | HX711 - DT                        | Dữ liệu từ Load Cell (Data Output)         |
| `PA12`            | HX711 - SCK                       | Clock cho HX711 (Serial Clock Input)       |
| `VCC 5V`          | HX711 - VCC                       | Cấp nguồn cho HX711 (5V)                   |
| `GND`             | GND chung                         | GND cho HX711, LED bar, và các module khác |
| `PG13`            | LED báo trạng thái                | Hiển thị khi đạt kỷ lục                    |
| `PE8 – PE13`      | Dãy 6 đèn LED                     | Hiển thị mức lực hoặc nháy hiệu ứng        |

TÍCH HỢP HỆ THỐNG
•	Phần cứng:
| Thành Phần           | Vai Trò                                                                 |
|----------------------|-------------------------------------------------------------------------|
| STM32F429I-DISC1     | Thiết bị xử lý trung tâm (MCU): điều khiển toàn bộ hệ thống             |
| Load Cell + HX711    | Cảm biến lực: đo lực tác động từ người chơi                             |
| 6 đèn LED            | Hiển thị trực quan mức lực thông qua số lượng đèn sáng                  |
| TFT LCD              | Hiển thị giá trị lực đo được, kỷ lục, giao diện trò chơi                |
| Flash trong STM32    | Lưu trữ giá trị lực lớn nhất (kỷ lục) qua nhiều lần tắt/mở hệ thống     |

•	Phần mềm:
| Phần Mềm               | Vai Trò                                                                      |
|------------------------|------------------------------------------------------------------------------|
| Firmware (main.c, HAL) | Điều khiển cảm biến, xử lý tín hiệu, điều khiển LED, LCD                     |
| HX711 driver           | Giao tiếp với module cảm biến lực HX711                                      |
| LCD BSP driver         | Giao tiếp với màn hình TFT LCD tích hợp trên STM32F429I-DISC1                |
| Game logic             | So sánh lực hiện tại với kỷ lục, cập nhật giao diện và điều khiển hiệu ứng   |

ĐẶC TẢ HÀM
•	void scale_init(void);   
/**
 @brief  Khởi tạo cảm biến lực (load cell + HX711) và các biến hệ thống. Gọi hàm hiệu chuẩn, thiết lập hệ số scale và trạng thái ban đầu.
 */
•	 void scale_calibrate(void);
/** 
@brief  Hiệu chuẩn cảm biến bằng cách lấy giá trị offset (tare). Giúp đảm bảo khi không có tải, giá trị đo là 0.
 */
•	 float scale_read_force(void);
/**
 @brief  Đọc giá trị lực từ HX711.
 @retval Giá trị lực đo được (đơn vị: Newton hoặc tuỳ theo hệ số SCALE_FACTOR).
 */
•	float scale_filter_force(float raw_force);  
/**
 @brief  Lọc nhiễu giá trị lực đo được bằng bộ lọc trung bình mũ (EMA).
 @param  raw_force  Giá trị lực chưa lọc (thô).
 @retval Giá trị lực đã lọc.
 */   
•	uint8_t scale_is_force_stable(float force);
/**
 @brief  Kiểm tra xem giá trị lực có ổn định (không dao động) không.
 @param  force  Giá trị lực hiện tại.
 @retval 1 nếu lực ổn định liên tục, 0 nếu chưa ổn định.
 */
•	void scale_display_force(float force, float max_force);
/**
 @brief  Hiển thị giá trị lực hiện tại và lực lớn nhất lên LED bar và UART.
 @param  force      Lực hiện tại.
 @param  max_force  Lực lớn nhất đã ghi nhận trong phiên chơi.
 */
•	void scale_handle_error(const char* error_msg);
/**
 @brief  Xử lý lỗi bằng cách nháy LED và gửi thông báo lỗi qua UART.
 @param  error_msg  Chuỗi thông báo lỗi.
 */
•	void LED_Control(uint8_t led_num, uint8_t state);
/**
 @brief  Điều khiển bật/tắt 1 LED trong dãy LED bar.
 @param  led_num  Số thứ tự LED (1 đến 6).
 @param  state    Trạng thái: 1 = bật, 0 = tắt.
 */
•	void DisplayNumber(uint8_t num);
/**
 @brief  Hiển thị số LED tương ứng với mức lực. Các LED từ 1 đến num sẽ được bật.
 @param  num  Số lượng LED cần bật (tối đa là 6).
 */
•	void BlinkLEDs(uint8_t times, uint16_t delay_ms);
/**
 @brief  Nhấp nháy tất cả LED bar trong một số lần nhất định.
 @param  times      Số lần nhấp nháy.
 @param  delay_ms   Thời gian delay giữa mỗi lần nhấp nháy (miligiây).
 */
KẾT QUẢ
