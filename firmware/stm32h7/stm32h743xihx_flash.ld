/**
 ******************************************************************************
 * @file    stm32h743xihx_flash.ld
 * @brief   Linker script cho STM32H743XIH6
 *            - Flash : 2 MB  @ 0x08000000
 *            - DTCM  : 128KB @ 0x20000000  (stack đặt ở đây - tốc độ cao nhất)
 *            - ITCM  : 64KB  @ 0x00000000  (dành cho code cực nhạy timing)
 *            - AXI SRAM (SRAM_D1): 512KB @ 0x24000000 (.data/.bss/heap)
 *            - SRAM1/2/3 (D2 domain): 128K+128K+32K, liên tiếp 0x30000000
 *              -> 32KB đầu tiên (0x30000000-0x30007FFF) dành riêng làm vùng
 *                 ".noncacheable" cho buffer DMA (map bởi MPU trong
 *                 startup_stm32h743xx.c: MPU_Config()).
 *            - SRAM4 (D3 domain): 64KB @ 0x38000000
 *            - Backup SRAM: 4KB @ 0x38800000
 *
 * LƯU Ý VỀ ĐỊA CHỈ: theo Reference Manual RM0433 (STM32H742/743/753/750),
 * SRAM2 thực tế nằm ở 0x30020000 và SRAM3 ở 0x30040000, Backup SRAM ở
 * 0x38800000 (không phải 0x30050000 / 0x3007C000 / 0x38000000 - 0x38000000
 * là SRAM4). Script này dùng đúng địa chỉ theo Reference Manual để đảm bảo
 * hoạt động chính xác trên phần cứng thật.
 ******************************************************************************
 */

ENTRY(Reset_Handler)

/* Đỉnh stack = đỉnh vùng DTCM RAM (0x20000000 + 128K = 0x20020000).
 * DTCM có độ trễ truy cập bằng 0 wait-state và không đi qua AXI bus matrix
 * -> tối ưu tốc độ cho stack (push/pop liên tục). */
_estack = 0x20000000 + 128K;

/* Kích thước heap/stack tối thiểu dự trù - dùng để linker cảnh báo (ASSERT)
 * nếu RAM không đủ chỗ. Có thể override bằng cách định nghĩa lại 2 symbol
 * này trước khi include file .ld (vd: qua -Wl,--defsym=_Min_Heap_Size=...). */
PROVIDE(_Min_Heap_Size  = 0x2000);   /* 8 KB */
PROVIDE(_Min_Stack_Size = 0x4000);   /* 16 KB */

MEMORY
{
  ITCMRAM     (xrw) : ORIGIN = 0x00000000, LENGTH = 64K
  FLASH       (rx)  : ORIGIN = 0x08000000, LENGTH = 2048K
  DTCMRAM     (xrw) : ORIGIN = 0x20000000, LENGTH = 128K
  RAM_AXI     (xrw) : ORIGIN = 0x24000000, LENGTH = 512K

  /* 32KB đầu SRAM1, dành riêng cho .noncacheable (DMA-safe buffer).
   * Base address 0x30000000 chia hết cho 32K -> hợp lệ với yêu cầu
   * "naturally aligned" của MPU ARMv7-M (bắt buộc base % size == 0). */
  RAM_NOCACHE (xrw) : ORIGIN = 0x30000000, LENGTH = 32K

  /* Phần còn lại của SRAM1 (96K) + SRAM2 (128K) + SRAM3 (32K) = 256K,
   * liên tục về mặt địa chỉ vật lý (0x30008000 - 0x30048000), dùng chung
   * cho mục đích tổng quát (buffer không cần DMA, bảng dữ liệu, ...). */
  RAM_D2      (xrw) : ORIGIN = 0x30008000, LENGTH = 256K

  RAM_D3      (xrw) : ORIGIN = 0x38000000, LENGTH = 64K   /* SRAM4 */
  RAM_BACKUP  (xrw) : ORIGIN = 0x38800000, LENGTH = 4K    /* Backup SRAM */
}

SECTIONS
{
  /* Vector Table - PHẢI ở đầu Flash (0x08000000) để CPU nạp đúng MSP/PC
   * khi thoát reset (hoặc để boot ROM nhảy vào đúng vị trí). */
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  } >FLASH

  /* Code chương trình */
  .text :
  {
    . = ALIGN(4);
    *(.text)
    *(.text*)
    *(.glue_7)
    *(.glue_7t)
    *(.eh_frame)
    KEEP (*(.init))
    KEEP (*(.fini))
    . = ALIGN(4);
    _etext = .;
  } >FLASH

  /* Hằng số / dữ liệu chỉ đọc */
  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)
    *(.rodata*)
    . = ALIGN(4);
  } >FLASH

  .ARM.extab :
  {
    *(.ARM.extab* .gnu.linkonce.armextab.*)
  } >FLASH

  .ARM :
  {
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
  } >FLASH

  .preinit_array :
  {
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
  } >FLASH

  .init_array :
  {
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
  } >FLASH

  .fini_array :
  {
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
  } >FLASH

  /* _sidata = địa chỉ LOAD (LMA) của .data trong FLASH - Reset_Handler
   * dùng symbol này làm điểm nguồn để copy sang RAM (VMA). */
  _sidata = LOADADDR(.data);

  /* .data: biến toàn cục có khởi tạo != 0. VMA đặt tại RAM_AXI (AXI SRAM),
   * LMA (nơi lưu giá trị khởi tạo thật) nằm trong FLASH ("AT>FLASH").
   * Muốn đổi sang DTCM: đổi "RAM_AXI" -> "DTCMRAM" ở dòng ">RAM_AXI" bên
   * dưới (và giảm tương ứng kích thước dành cho stack nếu cần). */
  .data :
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data)
    *(.data*)
    *(.axi_ram_data)      /* bien dung macro AXI_SRAM_DATA trong mem_attr.h */
    . = ALIGN(4);
    _edata = .;
  } >RAM_AXI AT> FLASH

  /* .itcm_text: code dat vao ITCM (macro ITCM_FUNC trong mem_attr.h).
   * ITCM la vung nho lenh rieng, 0 wait-state, khong tranh chap bus voi
   * D-Cache/DMA - danh cho ham cuc nhay timing (ISR tan so cao, dieu khien
   * dong co...). Giong .data: code nam trong Flash (LMA), duoc copy sang
   * ITCM (VMA) boi Reset_Handler truoc khi goi main(). */
  _siitcm = LOADADDR(.itcm_text);

  .itcm_text :
  {
    . = ALIGN(4);
    _sitcm = .;
    *(.itcm_text)
    . = ALIGN(4);
    _eitcm = .;
  } >ITCMRAM AT> FLASH

  /* .bss: biến toàn cục không khởi tạo / khởi tạo = 0, được zero-init
   * bởi Reset_Handler, không chiếm dung lượng trong Flash. */
  .bss :
  {
    . = ALIGN(4);
    _sbss = .;
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
    __bss_end__ = _ebss;
  } >RAM_AXI

  /* Vùng DMA-safe buffer: biến đặt attribute
   *   __attribute__((section(".noncacheable")))
   * sẽ rơi vào đây. Vùng vật lý này trùng khớp với region MPU cấu hình
   * trong MPU_Config() (32KB @ 0x30000000, Normal Non-cacheable Shareable)
   * -> CPU đọc/ghi không qua D-Cache, an toàn khi DMA truy cập trực tiếp
   * mà không cần SCB_CleanDCache()/InvalidateDCache() thủ công.
   * NOLOAD vì không cần giá trị khởi tạo từ Flash - Reset_Handler sẽ
   * zero-init vùng này bằng vòng lặp _snoncacheable..._enoncacheable. */
  .noncacheable (NOLOAD) :
  {
    . = ALIGN(32);
    _snoncacheable = .;
    *(.noncacheable)
    *(.noncacheable*)
    . = ALIGN(32);
    _enoncacheable = .;
  } >RAM_NOCACHE

  /* Kiểm tra vùng .noncacheable không vượt quá 32KB dành riêng cho MPU region */
  ASSERT(_enoncacheable - _snoncacheable <= 32K, "Vung .noncacheable vuot qua 32KB MPU region!")

  /* Heap (malloc). _end / end: symbol chuẩn newlib dùng làm điểm bắt đầu
   * heap cho _sbrk().
   *
   * CHỈ dự trù chỗ cho heap ở đây - KHÔNG cộng thêm _Min_Stack_Size vào
   * RAM_AXI như mẫu linker script gốc của ST (mẫu đó đúng cho các dòng
   * F4/F7 nơi stack và heap dùng chung 1 vùng RAM). Ở thiết kế này stack
   * thật sự nằm trong DTCMRAM (_estack), không phải RAM_AXI - nếu vẫn cộng
   * _Min_Stack_Size vào đây sẽ tạo ra 1 vùng "stack ma" chiếm oan chỗ của
   * .data/.bss/heap mà không tương ứng với bộ nhớ stack thực tế nào cả.
   * Việc kiểm tra ngân sách cho stack thật đã có ASSERT riêng bên dưới,
   * đối chiếu đúng với DTCMRAM. */
  ._user_heap :
  {
    . = ALIGN(8);
    PROVIDE ( end = . );
    PROVIDE ( _end = . );
    . = . + _Min_Heap_Size;
    . = ALIGN(8);
  } >RAM_AXI

  .ARM.attributes 0 : { *(.ARM.attributes) }
}

/* Đảm bảo stack (DTCM, tính từ _estack lùi xuống) và phần dữ liệu đã cấp
 * phát trong RAM_AXI không vượt quá dung lượng vật lý thật của mỗi vùng.
 * Linker sẽ tự báo lỗi "region ... overflowed" nếu SECTIONS ở trên vượt
 * quá LENGTH khai báo trong MEMORY, nên ASSERT dưới đây chỉ là lớp kiểm
 * tra bổ sung cho riêng ngân sách stack trong DTCM. */
ASSERT(_estack <= ORIGIN(DTCMRAM) + LENGTH(DTCMRAM), "Stack pointer vuot qua DTCM RAM!")
ASSERT(_Min_Stack_Size <= LENGTH(DTCMRAM), "_Min_Stack_Size lon hon toan bo DTCM RAM!")
