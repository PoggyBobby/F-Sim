/* Host replacement for the XC2000 ptypes_xe167.h, pre-included by the
 * Makefile (-include). The TASKING target has 16-bit int and 32-bit long;
 * on the host those are 32/64-bit, so the firmware's ubyte2/ubyte4 CAN and
 * ADC arithmetic would silently change width. Fixed-width types keep it
 * identical. Defining the original include guard skips the real header. */
#ifndef _PTYPES_H_XE167
#define _PTYPES_H_XE167 1

#include <stdint.h>

typedef uint8_t  ubyte1;
typedef uint16_t ubyte2;
typedef uint32_t ubyte4;
typedef int8_t   sbyte1;
typedef int16_t  sbyte2;
typedef int32_t  sbyte4;
typedef float    float4;
typedef unsigned char bool;

#ifndef FALSE
#define FALSE ((bool)0)
#endif
#ifndef TRUE
#define TRUE (!FALSE)
#endif
#ifndef NULL
#define NULL (0)
#endif

/* TASKING memory qualifiers used by APDB.h; meaningless on the host. */
#define __huge
#define __at(address)

/* Flash date stamped into the APDB by the TTTech build step
 * (build/builddate/getbuilddate.exe); there is no downloader here. */
#define RTS_TTC_FLASH_DATE_YEAR   0
#define RTS_TTC_FLASH_DATE_MONTH  0
#define RTS_TTC_FLASH_DATE_DAY    0
#define RTS_TTC_FLASH_DATE_HOUR   0
#define RTS_TTC_FLASH_DATE_MINUTE 0

#endif
