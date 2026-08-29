/* Implements clock_gettime() - rcutils/src/time_unix.c (built into libmicroros.a, see
 * firmware/stm32h7/microros/) calls it directly for both CLOCK_REALTIME and
 * CLOCK_MONOTONIC, and arm-none-eabi's nosys.specs provides no such syscall (bare metal,
 * no OS clock). Backed by Tick_GetMs() (TIM2 free-running ms counter, tick.h - KHONG
 * FreeRTOS o day, khac ban goc OUT_SAVE/testSTM/app/src/microros_time.c dung xTaskGetTickCount())
 * - monotonic tu luc boot, KHONG phai wall-clock (khong RTC tren board nay). Du dung cho
 * rcl/rclc noi bo (timeout, executor spin budget) - KHONG lam dung timestamp message
 * publish hay hien thi wall-clock cua `ros2 topic echo` cho dung.
 *
 * _POSIX_TIMERS/_POSIX_MONOTONIC_CLOCK/_POSIX_C_SOURCE (firmware/stm32h7/Makefile CFLAGS)
 * phai dinh nghia truoc <time.h> duoc include o bat ky dau trong app, neu khong newlib se
 * an prototype clock_gettime()/CLOCK_MONOTONIC di (xem microros/toolchain.cmake, dung
 * chinh xac cac define nay khi build libmicroros.a vi cung ly do). */
#include <time.h>

#include "tick.h"

int clock_gettime(clockid_t clock_id, struct timespec *tp)
{
    (void)clock_id;   /* chi co 1 nguon dong ho - dung chung tick cho ca 2 id */
    if (tp == NULL)
    {
        return -1;
    }

    const uint64_t ms = (uint64_t)Tick_GetMs();
    tp->tv_sec = (time_t)(ms / 1000U);
    tp->tv_nsec = (long)((ms % 1000U) * 1000000U);
    return 0;
}
