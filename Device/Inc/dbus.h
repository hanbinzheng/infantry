#ifndef __DBUS_H__
#define __DBUS_H__

#include "stdint.h"

#define SW_UP       0x01  // switch up
#define SW_MID      0x03  // switch mid
#define SW_DOWN     0x02  // switch down
#define DBUS_OFFLINE_TICK   200  // dbus offline time tick (ms)

#define DBUS_FRAME_LENGTH 18  // 18 byte a data frame
#define DBUS_RX_BUF_NUM 36  // double buffer dma

typedef struct {
    // normalized channel value (-660 ~ 660 -> -1 ~ 1)
    float ls_x;
    float ls_y;
    float rs_x;
    float rs_y;

    // switch value: 1, 2, 3
    uint8_t sw1;
    uint8_t sw2;

    // left wheel
    int16_t wheel;

    // mouse
    struct {
        // movement
        int16_t x;
        int16_t y;
        int16_t z; // no use

        // press: 1 release: 0
        uint8_t l;
        uint8_t r;
    } mouse;

    // keyboard
    /**********************************************************************************
     * keyboard: 15   14   13   12   11   10   9   8   7   6     5     4   3   2   1
     *            V    C    X	 Z    G    F   R   E   Q  CTRL SHIFT   D   A   S   W
     ************************************************************************************/
    union {
        uint16_t key_code; 
        struct {
            uint16_t w: 1;
            uint16_t s: 1;
            uint16_t a: 1;
            uint16_t d: 1;
            uint16_t shift: 1;
            uint16_t ctrl: 1;
            uint16_t q: 1;
            uint16_t e: 1;
            uint16_t r: 1;
            uint16_t f: 1;
            uint16_t g: 1;
            uint16_t z: 1;
            uint16_t x: 1;
            uint16_t c: 1;
            uint16_t v: 1;
            uint16_t b: 1;
        } key_bit;
    } keyboard;
} DbusData;

void dbus_data_interpret(uint8_t *buff, DbusData *dbus_data);


// global variables
extern DbusData dbus_data;
extern uint8_t DbusRxBuf[2][DBUS_FRAME_LENGTH];
extern uint32_t dbus_tick;

#endif //__DBUS_H__
