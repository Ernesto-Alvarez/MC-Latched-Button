/*
 * File:   main.c
 * Author: ealvarez
 *
 * Created on March 25, 2018, 12:44 PM
 */

// CONFIG
#pragma config FOSC = INTRCIO   // Oscillator Selection bits (INTOSC oscillator: I/O function on GP4/OSC2/CLKOUT pin, I/O function on GP5/OSC1/CLKIN)
#pragma config WDTE = OFF        // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = OFF      // Power-Up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // GP3/MCLR pin function select (GP3/MCLR pin function is digital I/O, MCLR internally tied to VDD)
#pragma config BOREN = ON       // Brown-out Detect Enable bit (BOD enabled)
#pragma config CP = OFF         // Code Protection bit (Program Memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)

#define _XTAL_FREQ = 4000000



#include <xc.h>
#include <stdint.h>

eeprom uint16_t timer_seconds = 15;
eeprom uint8_t hold_tenths = 25;
eeprom uint8_t abort_tenths = 5;
eeprom uint8_t led_persistence_tenths = 2;


void init(void)
{
    /*Timers*/
    
    /*Ports*/
    OPTION_REG = 0b00001000;        /*Enable Pull ups, no prescaler on TMR0*/
    INTCON     = 0b00000000;        /*No interrupts*/
    GPIO       = 0b000001;
    TRISIO     = 0b001110;
    WPU        = 0b000110;
    CMCON      = 0b00000111;        /*Comparator off*/
    VRCON      = 0b00000000;        /*Voltage reference off*/
    return;
}

void rt_loop(const uint16_t timer_seconds, const uint8_t hold_tenths, const uint8_t abort_tenths, const uint8_t led_persistence_tenths)
{
    
    
    uint8_t conditionals = 0;
    uint8_t led_state = 0;
    /*
       Bit 0: LED Timer
       Bit 1: Button
    */
    
    uint16_t led_ticks = 0;
    uint16_t led_timer = 0;
    uint8_t gpio_shadow = 0;
    
    led_ticks = led_persistence_tenths * 1667;
    
    /* Full RT loop. Time: 60*/
    asm("start_rt:");
    
       
    /*  Read GPIO (Time: 2)
        In:     GPIO
        Out:    gpio_shadow
     
     gpio_shadow = GPIO
     */
    
    asm("MOVF GPIO,w");
    asm("MOVWF rt_loop@gpio_shadow");
    
    /* Recovery from invalid LED states (Time: 12)
     In: led_state
     Out: led_state
     Notes: the code below each conditional should
        never be called under normal conditions
     
     Valid state list:
     state 0 (persistence timer off, button not pressed)
     state 1 (persistence timer on, button not pressed)
     state 2 (persistence timer off, button pressed)
     
     Invalid states:
     state 3 (persistence timer on, button pressed)
     states 4 and above (undefined)
     
     Recovery actions: Go to state 0, final state depends on button state
     */
    
    /*Check for undefined states*/
    asm("CLRF rt_loop@conditionals");
    asm("MOVF rt_loop@led_state,w");
    asm("ANDLW 248");
    asm("BTFSS STATUS,2");
    asm("BSF rt_loop@conditionals,0");
    
    /*Check for state 3*/
    asm("MOVF rt_loop@led_state,w");
    asm("ANDLW 3");
    asm("XORLW 3");
    asm("BTFSC STATUS,2");
    asm("BSF rt_loop@conditionals,0");
    
    /*Recovery code, use conditional bit 0*/
    asm("BTFSC rt_loop@conditionals,0");
    asm("CLRF rt_loop@led_state");
    
    /*Add any code that alerts of failure here, conditioned with bit 0*/
    
    /* Timers and in-state changes*/
    
    /* Timer decrement (time: 12)
     If timer is on (state 1), decrement timer.
     */
    
    /*Check for state 1 (conditional bit 0)*/
    asm("CLRF rt_loop@conditionals");
    asm("MOVLW 1");
    asm("XORWF rt_loop@led_state,w");
    asm("BTFSC STATUS,2");
    asm("BSF rt_loop@conditionals,0");
    
    /*State 1 AND byte 1 = 0 (conditional bit 1)*/
    asm("IORWF rt_loop@led_timer,w");
    asm("BTFSC STATUS,2");
    asm("BSF rt_loop@conditionals,1");
    
    /*Decrement timer*/
    asm("BTFSC rt_loop@conditionals,0");
    asm("DECF rt_loop@led_timer");
    asm("BTFSC rt_loop@conditionals,1");
    asm("DECF rt_loop@led_timer+1");
    
    
    
    
    /* State changes. Button has precedence*/
    
    /* Button press (time: 4)
     In: gpio_shadow bit 2 (button), led_state
     Out: led_state
     
     Button press changes state to state 2, irrespective of state
     
     Button is pressed when bit is 0, not pressed when bit is 1!!
     */
    asm("BTFSS rt_loop@gpio_shadow,2");
    asm("MOVLW 2");
    asm("BTFSS rt_loop@gpio_shadow,2");
    asm("MOVWF rt_loop@led_state");
    
    /* Button release (time: 19)
     Go to state 1 if state is 2 and button is not pressed, set timer on change
     Else, no action*/
    
    asm("CLRF rt_loop@conditionals");
    asm("MOVF rt_loop@led_state,w");
    asm("BTFSC rt_loop@gpio_shadow,2");
    asm("IORLW 4");
    asm("XORLW 6");
    asm("BTFSC STATUS,2");
    asm("BSF rt_loop@conditionals,0");
    
    asm("BTFSC rt_loop@conditionals,0");
    asm("MOVLW 1");
    asm("BTFSC rt_loop@conditionals,0");
    asm("MOVWF rt_loop@led_state");
    
    asm("BTFSC rt_loop@conditionals,0");
    asm("MOVF rt_loop@led_ticks,w");
    asm("BTFSC rt_loop@conditionals,0");
    asm("MOVWF rt_loop@led_timer");
    
    asm("BTFSC rt_loop@conditionals,0");
    asm("MOVF rt_loop@led_ticks+1,w");
    asm("BTFSC rt_loop@conditionals,0");
    asm("MOVWF rt_loop@led_timer+1");
    
    /* Timer end (time: 5)
     If timer = 0, clear its bit*/
    
    asm("CLRW");
    asm("IORWF rt_loop@led_timer,w");
    asm("IORWF rt_loop@led_timer+1,w");
    asm("BTFSC STATUS,2");
    asm("BCF rt_loop@led_state,0");
    
    /*Activate/deactivate LED (time: 4)
     State 0 = off
     else = on */
    
    asm("BSF rt_loop@gpio_shadow,4");
    asm("MOVF rt_loop@led_state,f");
    asm("BTFSC STATUS,2");
    asm("BCF rt_loop@gpio_shadow,4");
    
    /*Commit shadow GPIO to real (time: 2)*/
    
    asm("MOVF rt_loop@gpio_shadow,w");
    asm("MOVWF GPIO");
    
    
    /*Loop (time:2)*/
    asm("GOTO start_rt");
}


void main(void) {
    init();
    for (;;)
    {
        rt_loop(timer_seconds,hold_tenths,abort_tenths,led_persistence_tenths);
    }
}
