.syntax unified
.thumb

.global alarm_tick_asm

.extern g_millis
.extern g_state
.extern g_armed
.extern g_led_mode
.extern g_buzzer_gate
.extern g_sw2_edge
.extern g_sw2_raw
.extern g_sw3_raw
.extern g_alarm_start_ms
.extern g_alarm_stop_ms

.equ ST_DISARMED, 0
.equ ST_ARMED,    1
.equ ST_ALARM,    2

.equ DEBOUNCE_MS,  25
.equ LONGPRESS_MS, 600
.equ BEEP_ON_MS,   200
.equ BEEP_OFF_MS,  200

.bss
.align 2
deb_sw2_ct:     .space 4
deb_sw3_ct:     .space 4
sw2_stable:     .space 4
sw3_stable:     .space 4
sw2_prev:       .space 4
sw3_press_ms:   .space 4
beep_timer:     .space 4
.align 2

.text
.thumb_func
alarm_tick_asm:
    PUSH    {r4, lr}

    /* sample raw inputs */
    ldr     r0, =g_sw2_raw
    ldr     r0, [r0]
    ldr     r1, =g_sw3_raw
    ldr     r1, [r1]

    /* debounce SW2 */
    ldr     r2, =sw2_stable
    ldr     r3, [r2]
    cmp     r0, r3
    beq     deb_sw3
    ldr     r2, =deb_sw2_ct
    ldr     r3, [r2]
    adds    r3, r3, #1
    str     r3, [r2]
    cmp     r3, #DEBOUNCE_MS
    blt     deb_sw3
    ldr     r2, =sw2_stable
    str     r0, [r2]
    ldr     r2, =deb_sw2_ct
    movs    r3, #0
    str     r3, [r2]

deb_sw3:
    /* debounce SW3 */
    ldr     r2, =sw3_stable
    ldr     r3, [r2]
    cmp     r1, r3
    beq     sw3_press_check
    ldr     r2, =deb_sw3_ct
    ldr     r3, [r2]
    adds    r3, r3, #1
    str     r3, [r2]
    cmp     r3, #DEBOUNCE_MS
    blt     sw3_press_check
    ldr     r2, =sw3_stable
    str     r1, [r2]
    ldr     r2, =deb_sw3_ct
    movs    r3, #0
    str     r3, [r2]

sw3_press_check:
    /* latch press time */
    ldr     r2, =sw3_stable
    ldr     r2, [r2]
    cmp     r2, #1
    bne     sw3_release_check
    ldr     r3, =sw3_press_ms
    ldr     r12,[r3]
    cmp     r12,#0
    bne     sw3_release_check
    ldr     r12, =g_millis
    ldr     r12, [r12]
    str     r12, [r3]

sw3_release_check:
    /* if released, decide short/long */
    ldr     r2, =sw3_stable
    ldr     r2, [r2]
    cmp     r2, #0
    bne     door_logic
    ldr     r3, =sw3_press_ms
    ldr     r12,[r3]
    cmp     r12,#0
    beq     door_logic
    /* dt = now - press_ms */
    ldr     r0, =g_millis
    ldr     r0, [r0]
    subs    r0, r0, r12
    movs    r12,#0
    str     r12,[r3]
    ldr     r4, =LONGPRESS_MS
    cmp     r0, r4
    blt     sw3_short

    /* long: toggle armed/disarmed */
    ldr     r1, =g_armed
    ldr     r2, [r1]
    cmp     r2, #0
    bne     do_disarm
    movs    r2, #1
    str     r2, [r1]
    ldr     r3, =g_state
    ldr     r12,[r3]
    cmp     r12,#ST_ALARM
    beq     door_logic
    movs    r12,#ST_ARMED
    str     r12,[r3]
    ldr     r0, =g_led_mode
    movs    r2,#1
    str     r2,[r0]
    b       door_logic

do_disarm:
    movs    r2, #0
    str     r2, [r1]
    ldr     r3, =g_state
    movs    r12,#ST_DISARMED
    str     r12,[r3]
    ldr     r0, =g_buzzer_gate
    movs    r2,#0
    str     r2,[r0]
    ldr     r0, =g_led_mode
    movs    r2,#0
    str     r2,[r0]
    b       door_logic

sw3_short:
    /* short: stop alarm */
    ldr     r3, =g_state
    ldr     r12,[r3]
    cmp     r12,#ST_ALARM
    bne     door_logic
    ldr     r1, =g_armed
    ldr     r2,[r1]
    cmp     r2,#1
    bne     short_to_dis
    movs    r12,#ST_ARMED
    b       short_store
short_to_dis:
    movs    r12,#ST_DISARMED
short_store:
    str     r12,[r3]
    ldr     r0, =g_buzzer_gate
    movs    r2,#0
    str     r2,[r0]
    ldr     r0, =g_led_mode
    cmp     r12,#ST_ARMED
    bne     led_off_after_short
    movs    r2,#1
    b       led_store_after_short
led_off_after_short:
    movs    r2,#0
led_store_after_short:
    str     r2,[r0]
    ldr     r0, =g_millis
    ldr     r0, [r0]
    ldr     r1, =g_alarm_stop_ms
    str     r0, [r1]

door_logic:
    /* door event SW2 */
    ldr     r0, =g_sw2_edge
    ldr     r1, [r0]
    cmp     r1, #0
    beq     check_debounced
    movs    r1, #0
    str     r1, [r0]

    ldr     r1, =g_armed
    ldr     r2, [r1]
    cmp     r2, #1
    bne     beep_logic
    ldr     r1, =g_state
    ldr     r2, [r1]
    cmp     r2, #ST_ALARM
    beq     beep_logic
    movs    r2, #ST_ALARM
    str     r2, [r1]
    ldr     r1, =g_buzzer_gate
    movs    r2, #1
    str     r2, [r1]
    ldr     r1, =g_led_mode
    movs    r2, #2
    str     r2, [r1]
    ldr     r0, =g_millis
    ldr     r0, [r0]
    ldr     r1, =g_alarm_start_ms
    str     r0, [r1]
    ldr     r1, =beep_timer
    movs    r0, #0
    str     r0, [r1]
    b       beep_logic

check_debounced:
    ldr     r0, =sw2_stable
    ldr     r0, [r0]
    ldr     r1, =sw2_prev
    ldr     r2, [r1]
    cmp     r2, r0
    beq     beep_logic
    str     r0, [r1]
    cmp     r2, #0
    bne     beep_logic
    cmp     r0, #1
    bne     beep_logic
    ldr     r1, =g_armed
    ldr     r2, [r1]
    cmp     r2, #1
    bne     beep_logic
    ldr     r1, =g_state
    ldr     r2, [r1]
    cmp     r2, #ST_ALARM
    beq     beep_logic
    movs    r2, #ST_ALARM
    str     r2, [r1]
    ldr     r1, =g_buzzer_gate
    movs    r2, #1
    str     r2, [r1]
    ldr     r1, =g_led_mode
    movs    r2, #2
    str     r2, [r1]
    ldr     r0, =g_millis
    ldr     r0, [r0]
    ldr     r1, =g_alarm_start_ms
    str     r0, [r1]
    ldr     r1, =beep_timer
    movs    r0, #0
    str     r0, [r1]

beep_logic:
    /* beep gate while ALARM */
    ldr     r0, =g_state
    ldr     r0, [r0]
    cmp     r0, #ST_ALARM
    bne     done

    ldr     r1, =beep_timer
    ldr     r2, [r1]
    adds    r2, r2, #1
    str     r2, [r1]

    ldr     r3, =g_buzzer_gate
    ldr     r0, [r3]
    cmp     r0, #1
    bne     gate_off
    cmp     r2, #BEEP_ON_MS
    blt     done
    movs    r0, #0
    str     r0, [r3]
    movs    r2, #0
    str     r2, [r1]
    b       done

gate_off:
    cmp     r2, #BEEP_OFF_MS
    blt     done
    movs    r0, #1
    str     r0, [r3]
    movs    r2, #0
    str     r2, [r1]

done:
    POP     {r4, pc}
