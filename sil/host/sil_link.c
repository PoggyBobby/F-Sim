#include <stdio.h>
#include <stdlib.h>

#include "sil_link.h"

/* Rear inverter command frame IDs, extended format, as hardcoded in
 * canOutput_sendDebugMessage1 (canManager.c); payload is a big-endian
 * sbyte4 in data[0..3].
 *
 * On the S-diff branch these are 0x100 = RL, 0x101 = RR. They were the other
 * way round on sdiff-sil, which is what this file used to say — so every SIL
 * run was reading the left motor's command as the right one and vice versa,
 * mirroring the torque split. Re-check these against
 * canOutput_sendDebugMessage1 whenever the submodule moves. */
#define CMD_ID_CURRENT_RL  0x100
#define CMD_ID_CURRENT_RR  0x101

/* Duty-cycle command frames. No path on the S-diff branch emits these
 * (powertrainControl.c commands milliamps only), so mode 1 never fires there;
 * kept for the sdiff-sil-era duty path and any future one. */
#define CMD_ID_DUTY_RL     0x01
#define CMD_ID_DUTY_RR     0x00

SilInputs sil_in;

static int    cmd_mode = 0;          /* 0 none, 1 duty, 2 current (mA) */
static sbyte4 cmd_RL = 0;
static sbyte4 cmd_RR = 0;

void sil_link_read(void)
{
    char line[256];
    SilInputs f;

    cmd_mode = 0;                      /* a new cycle: nothing sent yet */
    cmd_RL = cmd_RR = 0;

    if (fgets(line, sizeof line, stdin) == NULL)
        exit(0);                       /* simulator closed the pipe: done */

    if (sscanf(line, "S %lf %lf %lf %lf %lf %lf %lf %lf %lf",
               &f.t_s, &f.apps_pct, &f.bps_bar, &f.handwheel_deg,
               &f.rpm_RL, &f.rpm_RR, &f.yaw_rate, &f.ax, &f.ay) == 9)
        sil_in = f;                    /* malformed line: keep last frame */
}

void sil_link_write(ubyte4 t_vcu_us)
{
    printf("T %.3f %d %ld %ld\n", t_vcu_us / 1e6, cmd_mode,
           (long)cmd_RL, (long)cmd_RR);
    fflush(stdout);
}

void sil_link_capture_can(const IO_CAN_DATA_FRAME *frame)
{
    sbyte4 value = (sbyte4)(((ubyte4)frame->data[0] << 24) |
                            ((ubyte4)frame->data[1] << 16) |
                            ((ubyte4)frame->data[2] << 8)  |
                             (ubyte4)frame->data[3]);

    if (frame->id_format != IO_CAN_EXT_FRAME)
        return;
    switch (frame->id) {
    case CMD_ID_DUTY_RL:    cmd_mode = 1; cmd_RL = value; break;
    case CMD_ID_DUTY_RR:    cmd_mode = 1; cmd_RR = value; break;
    case CMD_ID_CURRENT_RL: cmd_mode = 2; cmd_RL = value; break;
    case CMD_ID_CURRENT_RR: cmd_mode = 2; cmd_RR = value; break;
    default: break;
    }
}
