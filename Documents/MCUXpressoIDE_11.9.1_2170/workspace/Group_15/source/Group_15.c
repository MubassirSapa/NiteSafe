// Group_15 NiteSafe - Interrupt + ASM state machine
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_gpio.h"
#include "fsl_common.h"

#include "app.h"

// Pins (FRDM-K66F)
#define SW2_GPIO       GPIOD
#define SW2_PIN        11u          // PTD11 (SW2) active-low
#define SW3_GPIO       GPIOA
#define SW3_PIN        10u          // PTA10 (SW3) active-low
#define LEDR_GPIO      GPIOC
#define LEDR_PIN       9u           // PTC9 RED (active-low)
#define LEDG_GPIO      GPIOE
#define LEDG_PIN       6u           // PTE6 GREEN (active-low)
#define BUZ_GPIO       GPIOB
#define BUZ_PIN        18u          // PTB18 (D8) active-high buzzer

// Shared state (C <-> ASM)
volatile uint32_t g_millis = 0;
volatile uint8_t  g_state  = ST_DISARMED;
volatile uint8_t  g_armed  = 0;
volatile uint8_t  g_led_mode    = 0;
volatile uint8_t  g_buzzer_gate = 0;
volatile uint8_t  g_sw2_edge = 0;
volatile uint8_t  g_sw2_raw  = 0;
volatile uint8_t  g_sw3_raw  = 0;
volatile uint32_t g_alarm_start_ms = 0;
volatile uint32_t g_alarm_stop_ms  = 0;

// Helpers
static inline void gpio_out(GPIO_Type *gpio, uint32_t pin, uint8_t high) {
    if (high) gpio->PSOR = (1u << pin); else gpio->PCOR = (1u << pin);
}
static inline uint8_t gpio_in(GPIO_Type *gpio, uint32_t pin) {
    return (gpio->PDIR & (1u << pin)) ? 1u : 0u;
}
static inline void led_out(GPIO_Type *gpio, uint32_t pin, uint8_t on) {
    if (on) gpio->PCOR = (1u << pin); else gpio->PSOR = (1u << pin);
}
static uint32_t get_bus_clock_hz(void) {
    uint32_t div = ((SIM->CLKDIV1 & SIM_CLKDIV1_OUTDIV2_MASK) >> SIM_CLKDIV1_OUTDIV2_SHIFT) + 1u;
    return SystemCoreClock / div;
}

static void board_init_outputs(void) {
    SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTE_MASK;
    PORTC->PCR[LEDR_PIN] = PORT_PCR_MUX(1);
    PORTE->PCR[LEDG_PIN] = PORT_PCR_MUX(1);
    PORTB->PCR[BUZ_PIN]  = PORT_PCR_MUX(1);
    LEDR_GPIO->PDDR |= (1u << LEDR_PIN);
    LEDG_GPIO->PDDR |= (1u << LEDG_PIN);
    BUZ_GPIO->PDDR  |= (1u << BUZ_PIN);
    led_out(LEDR_GPIO, LEDR_PIN, 0);
    led_out(LEDG_GPIO, LEDG_PIN, 0);
    gpio_out(BUZ_GPIO, BUZ_PIN, 0);
}

static void board_init_inputs(void) {
    SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTD_MASK;
    // Mux to GPIO, pull-up, falling-edge interrupt
    PORTD->PCR[SW2_PIN] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK | PORT_PCR_IRQC(0xA);
    PORTA->PCR[SW3_PIN] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK | PORT_PCR_IRQC(0xA);
    PORTD->ISFR = (1u << SW2_PIN);
    PORTA->ISFR = (1u << SW3_PIN);
}

static void pit0_start_1khz(void) {
    SIM->SCGC6 |= SIM_SCGC6_PIT_MASK;
    PIT->MCR = 0;
    uint32_t bus = get_bus_clock_hz();
    uint32_t ldval = (bus / 1000u) - 1u;
    PIT->CHANNEL[0].LDVAL = ldval;
    PIT->CHANNEL[0].TCTRL = PIT_TCTRL_TIE_MASK | PIT_TCTRL_TEN_MASK;
}

// Outputs from flags
static void service_outputs(void) {
    uint8_t lm = g_led_mode;
    if (lm == 0) {
        led_out(LEDG_GPIO, LEDG_PIN, 0);
        led_out(LEDR_GPIO, LEDR_PIN, 0);
    } else if (lm == 1) {
        led_out(LEDG_GPIO, LEDG_PIN, 1);
        led_out(LEDR_GPIO, LEDR_PIN, 0);
    } else {
        uint8_t on = ((g_millis / 125u) & 1u) ? 1u : 0u; // blink
        led_out(LEDR_GPIO, LEDR_PIN, on);
        led_out(LEDG_GPIO, LEDG_PIN, 0);
    }
    gpio_out(BUZ_GPIO, BUZ_PIN, (g_buzzer_gate != 0));
}

// Interrupts
void PORTD_IRQHandler(void) {
    if (PORTD->ISFR & (1u << SW2_PIN)) {
        PORTD->ISFR = (1u << SW2_PIN);
        g_sw2_edge = 1;
        g_armed = 1;
        g_state = ST_ALARM;
        g_led_mode = 2;
        g_buzzer_gate = 1;
        g_alarm_start_ms = g_millis;
    } else {
        uint32_t isfr = PORTD->ISFR;
        if (isfr) PORTD->ISFR = isfr;
    }
}
void PORTA_IRQHandler(void) {
    uint32_t mask = (1u << SW3_PIN);
    if (PORTA->ISFR & mask) PORTA->ISFR = mask;
    else {
        uint32_t isfr = PORTA->ISFR;
        if (isfr) PORTA->ISFR = isfr;
    }
}
void PIT0_IRQHandler(void) {
    PIT->CHANNEL[0].TFLG = PIT_TFLG_TIF_MASK;
    g_millis++;
    g_sw2_raw = gpio_in(SW2_GPIO, SW2_PIN) ? 0u : 1u;
    g_sw3_raw = gpio_in(SW3_GPIO, SW3_PIN) ? 0u : 1u;
    // Fallback edge detect if PORTD IRQ is missed
    static uint8_t prev_sw2 = 0;
    if ((prev_sw2 == 0u) && (g_sw2_raw == 1u)) {
        g_sw2_edge = 1;
    }
    prev_sw2 = g_sw2_raw;
    alarm_tick_asm();
}

// Main
int main(void) {
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
    board_init_outputs();
    board_init_inputs();
    pit0_start_1khz();
    __enable_irq();
    NVIC_EnableIRQ(PORTD_IRQn);
    NVIC_EnableIRQ(PORTA_IRQn);
    NVIC_EnableIRQ(PIT0_IRQn);

    g_state = ST_DISARMED;   // start disarmed
    g_armed = 0;
    g_led_mode = 0;          // LEDs off
    g_buzzer_gate = 0;

    while (1) {
        service_outputs();
        __WFI();
    }
}
