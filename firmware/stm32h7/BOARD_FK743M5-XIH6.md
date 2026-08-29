# Board FANKE FK743M5-XIH6 — Tài liệu tham khảo

Ghi chú: tài liệu này tổng hợp từ tài liệu chính thức của Zephyr Project
(nhà sản xuất board — FANKE Technology — không tự phát hành datasheet công
khai đầy đủ, Zephyr là nguồn chính thức đáng tin cậy nhất hiện có). Các mục
đánh dấu **"⚠ Chưa xác nhận"** là thông tin tôi không tìm được nguồn đủ tin
cậy — bạn nên tự kiểm tra trực tiếp trên board (đo đạc, đọc silkscreen) thay
vì tin tuyệt đối.

## 1. Tổng quan

| Thuộc tính | Giá trị |
|---|---|
| Tên board | FK743M5-XIH6 |
| Nhà sản xuất | FANKE Technology Co., Ltd. |
| MCU | STM32H743XIH6 |
| Gói chip (package) | TFBGA-265 |
| Debugger onboard | **Không có** — bắt buộc dùng ST-Link/J-Link rời qua SWD |
| Hỗ trợ Zephyr RTOS | Có (board tên `fk743m5_xih6`), nhưng ghi chú "not actively maintained" |

## 2. Vi điều khiển STM32H743XIH6

| Thuộc tính | Giá trị |
|---|---|
| Lõi | Arm Cortex-M7 (có FPU double-precision) |
| Xung nhịp tối đa | 480 MHz |
| Flash nội | 2048 KB (2 MB), địa chỉ `0x08000000` |
| SRAM nội | 1 MB, chia thành: |
| — ITCM RAM | 64 KB @ `0x00000000` |
| — DTCM RAM | 128 KB @ `0x20000000` |
| — AXI SRAM (user SRAM) | 512 KB @ `0x24000000` (tài liệu Zephyr gọi là "864 KB user SRAM" tính gộp cả SRAM1/2/3 D2 domain) |
| — SRAM1/SRAM2/SRAM3 (D2 domain) | 128 KB + 128 KB + 32 KB, liên tục từ `0x30000000` |
| — SRAM4 (D3 domain) | 64 KB @ `0x38000000` |
| — Backup SRAM | 4 KB @ `0x38800000` |

Bản đồ bộ nhớ này đã được ánh xạ chính xác trong file
[`stm32h743xihx_flash.ld`](stm32h743xihx_flash.ld) của project — 32 KB đầu
SRAM1 được dành riêng làm vùng `.noncacheable` cho buffer DMA (xem
`MPU_Config()` trong [`startup_stm32h743xx.c`](startup_stm32h743xx.c)).

## 3. Hệ thống Clock

| Nguồn clock | Tần số |
|---|---|
| HSE (thạch anh ngoài chính) | **25 MHz** |
| LSI/HSI nội | Có sẵn trong chip (không cần linh kiện ngoài) |
| PLL chính | Cấu hình được tối đa ra 480 MHz cho CPU |
| LSE (thạch anh RTC 32.768 kHz) | ⚠ Chưa xác nhận có được hàn sẵn trên board hay chỉ có chỗ (footprint) trống |

**Lưu ý quan trọng:** hàm `SystemInit()` trong `startup_stm32h743xx.c` của
project hiện tại **chỉ là hàm rỗng (placeholder)** — nghĩa là ngay sau khi
nạp code, chip đang chạy bằng **HSI nội mặc định (~64 MHz)**, chưa dùng đến
thạch anh 25 MHz này. Muốn chạy đúng tốc độ tối đa 480 MHz, cần tự viết cấu
hình PLL (HSE 25 MHz → PLL1) trong `SystemInit()`.

## 4. Ngoại vi có sẵn trên board

| Ngoại vi | Chi tiết | Chân MCU |
|---|---|---|
| LED người dùng | 1 LED (màu xanh dương), **active LOW** | PC13 |
| Nút bấm RESET | Reset cứng MCU (chân NRST) | NRST |
| Nút bấm BOOT | Chọn chế độ boot (BOOT0) | BOOT0 |
| USB | USB OTG Full-Speed **và** High-Speed | PA11/PA12 (OTG FS) |
| Khe thẻ nhớ | 1 khe micro SD (SDMMC) | — |
| Giao tiếp Camera | 1 cổng DCMI | — |
| Giao tiếp màn hình | 1 cổng LCD qua SPI5 | PE11 / PE12 / PE14 |
| Flash ngoài (QSPI) | Winbond **W25Q64** (64 Mbit = 8 MB), Quad-SPI, 40 MHz | PF6, PF7, PF8, PF9, PF10, PG6 (CS) |
| SDRAM ngoài (FMC) | Có mạch điều khiển FMC/SDRAM đấu ra chân PD/PE/PF/PG | ⚠ Chưa xác nhận dung lượng/chip cụ thể — tài liệu chỉ nêu tên gọi thương mại board là "16-bit SDRAM Core Board" |
| Số chân GPIO đưa ra header | 83 chân | — |
| Timer | 1 HRTIM (độ phân giải 2.1 ns), 2 timer 32-bit, 17 timer 16-bit | — |

**Cổng USB-C trên board KHÔNG dùng để nạp code** — đó là USB OTG nối thẳng
MCU (PA11/PA12), chỉ dùng khi ứng dụng của bạn tự triển khai USB Device/Host.

## 5. Kết nối Debug (SWD)

Board không có ST-Link tích hợp — bắt buộc nối ST-Link/J-Link rời qua header
SWD 4 chân trên board:

| Chân trên board | Ý nghĩa | Nối với ST-Link |
|---|---|---|
| DIO | SWDIO (PA13) | SWDIO |
| CLK | SWCLK (PA14) | SWCLK |
| GND | Mass chung | GND |
| 5V | Rail nguồn 5V (lấy từ VBUS cổng USB-C), **không phải mức logic debug** | Chỉ nối nếu muốn ST-Link cấp nguồn cho board thay vì cắm USB-C riêng — **không nối đồng thời cả 2 nguồn** |

Cách nạp code sau khi nối xong: xem [`Makefile`](Makefile), lệnh
`st-flash write build/firmware.bin 0x08000000`.

## 6. Bảng ánh xạ chân mặc định (theo cấu hình tham khảo của Zephyr)

| Ngoại vi | Chân | Công dụng |
|---|---|---|
| USART1 | PA9 (TX), PA10 (RX) | Cổng serial console, baud mặc định 115200 |
| SPI5 | PE11, PE12, PE14 | Giao tiếp màn hình LCD |
| QUADSPI | PF6, PF7, PF8, PF9, PF10, PG6 | Giao tiếp Flash W25Q64 ngoài |
| USB OTG FS | PA11, PA12 | USB Device/Host |
| LED | PC13 | LED người dùng (active LOW) |

Đây là cấu hình chân **được Zephyr đề xuất sử dụng**, không phải ràng buộc
phần cứng cố định (trừ SWDIO/SWCLK/LED là cố định theo thiết kế board) — bạn
vẫn có thể dùng UART/SPI/I2C khác trên 83 chân GPIO đưa ra nếu ứng dụng cần.

## 7. Những gì CHƯA xác nhận được (nên tự kiểm tra)

- Dung lượng và part number chính xác của chip SDRAM ngoài.
- Board có hàn sẵn thạch anh LSE 32.768 kHz hay không.
- Kích thước vật lý board, layout đầy đủ của header 83 chân GPIO (P1/P2/P3...).
- Điện áp input chính xác qua USB-C (thường 5V chuẩn USB, nhưng chưa thấy tài liệu ghi rõ dòng tối đa).

## 8. Nguồn tham khảo

- [Zephyr Project — FK743M5-XIH6 board documentation](https://docs.zephyrproject.org/latest/boards/fanke/fk743m5_xih6/doc/index.html)
- [Zephyr GitHub — fk743m5_xih6.dts (devicetree gốc)](https://github.com/zephyrproject-rtos/zephyr/blob/main/boards/fanke/fk743m5_xih6/fk743m5_xih6.dts)
- [Zephyr GitHub — PR thêm board FK743M5-XIH6](https://github.com/zephyrproject-rtos/zephyr/pull/86893)
