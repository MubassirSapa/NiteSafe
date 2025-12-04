//ChatGPT helped define the shared C/ASM interface: fixed-width types, volatile externs,
// and alarm state enum for cross-language consistency.


#ifndef APP_H
#define APP_H

#include <stdint.h>

// States
typedef enum { ST_DISARMED = 0, ST_ARMED = 1, ST_ALARM = 2 } app_state_t;

// Shared (C <-> ASM)
extern volatile uint32_t g_millis;

extern volatile uint8_t  g_state;        // ST_*
extern volatile uint8_t  g_armed;        // 0/1

extern volatile uint8_t  g_led_mode;     // 0=off, 1=green, 2=red_blink
extern volatile uint8_t  g_buzzer_gate;  // 0/1

extern volatile uint8_t  g_sw2_edge;     // set in SW2 ISR
extern volatile uint8_t  g_sw2_raw;      // sampled in PIT (1=pressed)
extern volatile uint8_t  g_sw3_raw;      // sampled in PIT (1=pressed)

extern volatile uint32_t g_alarm_start_ms;
extern volatile uint32_t g_alarm_stop_ms;

// Tunables (ms)
#define DEBOUNCE_MS      (25u)
#define LONGPRESS_MS     (600u)
#define BEEP_ON_MS       (200u)
#define BEEP_OFF_MS      (200u)

void alarm_tick_asm(void);  // 1 kHz tick

#endif
