/* Host stand-ins for the TTTech HY-TTC 60 IO library (lib/xc2000_ttc60.lib).
 *
 * Only the functions the firmware actually calls are provided. Every one
 * of them succeeds (IO_E_OK), outputs read as zero and "fresh", writes are
 * accepted and dropped. The one piece of behaviour that is NOT a no-op is
 * the real-time clock: IO_Driver_TaskEnd() advances it by one VCU cycle,
 * and every IO_RTC_GetTimeUS() poll by 1 µs, so the firmware's own
 * `while (IO_RTC_GetTimeUS(start) < N)` pacing loops terminate — after one
 * cycle when N is the cycle time, after N polls when the loop waits with
 * no cycle inside it (the ADC settle loop in initializations.c) — and the
 * main loop runs one iteration per cycle instead of spinning on wall-clock
 * time.
 *
 * The task hooks and the CAN write path are wired to sil_link.c: a sensor
 * frame is read from the simulator at the start of every cycle and the
 * torque frame written at its end. */

#include "IO_Driver.h"
#include "IO_RTC.h"
#include "IO_DIO.h"
#include "IO_ADC.h"
#include "IO_PWM.h"
#include "IO_PWD.h"
#include "IO_POWER.h"
#include "IO_CAN.h"
#include "IO_UART.h"

#include "sil_link.h"

#define SIL_CYCLE_US 10000UL   /* the firmware's main-loop period */
#define SIL_POLL_US  1UL       /* time a busy-wait poll is taken to cost */

static ubyte4 rtc_now_us = 0;

/* ── driver / task ─────────────────────────────────────────────────── */
IO_ErrorType IO_Driver_Init(const IO_DRIVER_SAFETY_CONF *const safety_conf)
{
    (void)safety_conf;
    return IO_E_OK;
}

IO_ErrorType IO_Driver_TaskBegin(void)
{
    sil_link_read();
    return IO_E_OK;
}

IO_ErrorType IO_Driver_TaskEnd(void)
{
    rtc_now_us += SIL_CYCLE_US;
    sil_link_write(rtc_now_us);
    return IO_E_OK;
}

/* ── real-time clock ───────────────────────────────────────────────── */
IO_ErrorType IO_RTC_StartTime(ubyte4 *const timestamp)
{
    *timestamp = rtc_now_us;
    return IO_E_OK;
}

ubyte4 IO_RTC_GetTimeUS(ubyte4 timestamp)
{
    rtc_now_us += SIL_POLL_US;
    return rtc_now_us - timestamp;
}

/* ── digital I/O ───────────────────────────────────────────────────── */
IO_ErrorType IO_DO_Init(ubyte1 do_channel)
{
    (void)do_channel;
    return IO_E_OK;
}

IO_ErrorType IO_DO_Set(ubyte1 do_channel, bool do_value)
{
    (void)do_channel; (void)do_value;
    return IO_E_OK;
}

IO_ErrorType IO_DI_Init(ubyte1 di_channel, ubyte1 mode)
{
    (void)di_channel; (void)mode;
    return IO_E_OK;
}

IO_ErrorType IO_DI_DeInit(ubyte1 di_channel)
{
    (void)di_channel;
    return IO_E_OK;
}

IO_ErrorType IO_DI_Get(ubyte1 di_channel, bool *const di_value)
{
    (void)di_channel;
    *di_value = FALSE;
    return IO_E_OK;
}

/* ── analog inputs ─────────────────────────────────────────────────── */
IO_ErrorType IO_ADC_ChannelInit(ubyte1 adc_channel, ubyte1 type, ubyte1 range,
                                ubyte1 pupd, ubyte1 sensor_supply,
                                IO_ADC_SAFETY_CONF const *const safety_conf)
{
    (void)adc_channel; (void)type; (void)range; (void)pupd;
    (void)sensor_supply; (void)safety_conf;
    return IO_E_OK;
}

IO_ErrorType IO_ADC_ChannelDeInit(ubyte1 adc_channel)
{
    (void)adc_channel;
    return IO_E_OK;
}

IO_ErrorType IO_ADC_Get(ubyte1 adc_channel, ubyte2 *const adc_value,
                        bool *const fresh)
{
    (void)adc_channel;
    *adc_value = 0;
    *fresh = TRUE;
    return IO_E_OK;
}

/* ── PWM outputs / pulse inputs ────────────────────────────────────── */
IO_ErrorType IO_PWM_Init(ubyte1 pwm_channel, ubyte2 frequency, bool polarity,
                         bool cur_measurement, ubyte1 cur_channel,
                         bool diag_margin,
                         IO_PWM_SAFETY_CONF const *const safety_conf)
{
    (void)pwm_channel; (void)frequency; (void)polarity; (void)cur_measurement;
    (void)cur_channel; (void)diag_margin; (void)safety_conf;
    return IO_E_OK;
}

IO_ErrorType IO_PWM_SetDuty(ubyte1 pwm_channel, ubyte2 duty_cycle,
                            ubyte4 *const duty_cycle_fb)
{
    (void)pwm_channel; (void)duty_cycle;
    if (duty_cycle_fb) *duty_cycle_fb = 0;
    return IO_E_OK;
}

IO_ErrorType IO_PWD_ComplexInit(ubyte1 timer_channel, ubyte1 pulse_mode,
                                ubyte1 freq_mode, ubyte1 timer_res,
                                ubyte1 capture_count, ubyte1 threshold,
                                ubyte1 pupd,
                                IO_PWD_CPLX_SAFETY_CONF const *const safety_conf)
{
    (void)timer_channel; (void)pulse_mode; (void)freq_mode; (void)timer_res;
    (void)capture_count; (void)threshold; (void)pupd; (void)safety_conf;
    return IO_E_OK;
}

IO_ErrorType IO_PWD_ComplexGet(ubyte1 timer_channel, ubyte2 *const frequency,
                               ubyte4 *const pulse_width,
                               IO_PWD_PULSE_SAMPLES *const pulse_samples)
{
    (void)timer_channel; (void)pulse_samples;
    *frequency = 0;
    *pulse_width = 0;
    return IO_E_OK;
}

IO_ErrorType IO_PWD_PulseInit(ubyte1 timer_channel, ubyte1 pulse_mode)
{
    (void)timer_channel; (void)pulse_mode;
    return IO_E_OK;
}

IO_ErrorType IO_PWD_PulseGet(ubyte1 timer_channel, ubyte4 *const pulse_width)
{
    (void)timer_channel;
    *pulse_width = 0;
    return IO_E_OK;
}

/* ── sensor supplies ───────────────────────────────────────────────── */
IO_ErrorType IO_POWER_Set(ubyte1 pin, ubyte1 mode)
{
    (void)pin; (void)mode;
    return IO_E_OK;
}

/* ── CAN ───────────────────────────────────────────────────────────── */
IO_ErrorType IO_CAN_Init(ubyte1 channel, ubyte2 baudrate, ubyte1 tseg1,
                         ubyte1 tseg2, ubyte1 sjw)
{
    (void)channel; (void)baudrate; (void)tseg1; (void)tseg2; (void)sjw;
    return IO_E_OK;
}

IO_ErrorType IO_CAN_ConfigFIFO(ubyte1 *const handle, ubyte1 channel,
                               ubyte1 size, ubyte1 mode, ubyte1 id_format,
                               ubyte4 id, ubyte4 ac_mask)
{
    (void)size; (void)id_format; (void)id; (void)ac_mask;
    *handle = (ubyte1)(channel * 2 + mode);
    return IO_E_OK;
}

IO_ErrorType IO_CAN_ReadFIFO(ubyte1 handle, IO_CAN_DATA_FRAME *const buffer,
                             ubyte1 buffer_size, ubyte1 *const rx_frames)
{
    (void)handle; (void)buffer; (void)buffer_size;
    *rx_frames = 0;
    return IO_E_OK;
}

IO_ErrorType IO_CAN_WriteFIFO(ubyte1 handle, const IO_CAN_DATA_FRAME *const data,
                              ubyte1 tx_length)
{
    ubyte1 i;
    (void)handle;
    for (i = 0; i < tx_length; i++)
        sil_link_capture_can(&data[i]);
    return IO_E_OK;
}

IO_ErrorType IO_CAN_WriteMsg(ubyte1 handle, const IO_CAN_DATA_FRAME *const data)
{
    (void)handle;
    sil_link_capture_can(data);
    return IO_E_OK;
}

/* ── UART ──────────────────────────────────────────────────────────── */
IO_ErrorType IO_UART_Task(void)
{
    return IO_E_OK;
}
