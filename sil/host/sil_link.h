/* Frame exchange between the host-built VCU and the simulator.
 *
 * One text line each way per VCU cycle, over stdin/stdout:
 *
 *   sim -> VCU  (read at IO_Driver_TaskBegin, blocks until it arrives)
 *     S <t_s> <apps_pct> <bps_bar> <handwheel_deg> <rpm_RL> <rpm_RR>
 *       <yaw_rate_rad_s> <ax_m_s2> <ay_m_s2>
 *
 *   VCU -> sim  (written at IO_Driver_TaskEnd)
 *     T <t_vcu_s> <mode> <cmd_RL> <cmd_RR>
 *
 * The blocking read is what synchronises the two: the firmware's main loop
 * cannot run a cycle until the simulator has sent that cycle's sensors,
 * and the simulator cannot advance until the VCU has answered.
 *
 * mode / cmd are what the firmware put on the rear inverters' command
 * frames THIS cycle (canOutput_sendDebugMessage1): mode 1 = duty cycle in
 * VESC wire units (fraction x 100000), mode 2 = current in mA, mode 0 =
 * no command frame sent this cycle. No scaling happens on this side.
 * t_vcu_s is the firmware's own clock, which starts at power-up; the sim
 * uses it to wait out the boot sequence before t = 0.
 *
 * The sensor frame is stored in `sil_in` for the IO stubs to serve. */

#ifndef SIL_LINK_H
#define SIL_LINK_H

#include "IO_Driver.h"
#include "IO_CAN.h"

typedef struct {
    double t_s;
    double apps_pct;
    double bps_bar;
    double handwheel_deg;
    double rpm_RL;
    double rpm_RR;
    double yaw_rate;
    double ax;
    double ay;
} SilInputs;

extern SilInputs sil_in;

void sil_link_read(void);
void sil_link_write(ubyte4 t_vcu_us);
void sil_link_capture_can(const IO_CAN_DATA_FRAME *frame);

#endif
