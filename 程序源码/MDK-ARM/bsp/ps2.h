#ifndef __PS2_H
#define __PS2_H

#include "mytype.h"

typedef struct {
    u8  id;
    u8  btn1;      /* bit7-0: LEFT,DOWN,RIGHT,UP,START,R3,L3,SELECT (0=pressed) */
    u8  btn2;      /* bit7-0: SQR,X,CIR,TRI,R1,L1,R2,L2 (0=pressed) */
    u8  joy_rx;    /* 0=left, 0x80=center, 0xFF=right */
    u8  joy_ry;    /* 0=up,   0x80=center, 0xFF=down */
    u8  joy_lx;    /* 0=left, 0x80=center, 0xFF=right */
    u8  joy_ly;    /* 0=up,   0x80=center, 0xFF=down */
} ps2_state_t;

void ps2_init(void);
u8   ps2_read(ps2_state_t *state);

/* btn1 bits (0 = pressed) */
#define PS2_SELECT  0x01
#define PS2_L3      0x02
#define PS2_R3      0x04
#define PS2_START   0x08
#define PS2_UP      0x10
#define PS2_RIGHT   0x20
#define PS2_DOWN    0x40
#define PS2_LEFT    0x80

/* btn2 bits (0 = pressed) */
#define PS2_L2      0x01
#define PS2_R2      0x02
#define PS2_L1      0x04
#define PS2_R1      0x08
#define PS2_CIR     0x10
#define PS2_TRI     0x20
#define PS2_X       0x40
#define PS2_SQR     0x80

#endif
