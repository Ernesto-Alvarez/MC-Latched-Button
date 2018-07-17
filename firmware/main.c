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

eeprom uint16_t timer_seconds = 50;
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
    /*LED Variables*/
    uint8_t led_state = 0;
    /*
       Bit 0: LED Timer
       Bit 1: Button
    */
    
    uint16_t led_ticks = 0;
    uint16_t led_timer = 0;
    uint8_t gpio_shadow = 0;
    
    /*Relay variables*/
    uint8_t relay_state = 0;
    /*
        Bit 0: Relay Timer
        Bit 1: Hold Timer
        Bit 2: Abort Timer
        Bit 3: Button
     */
    
    uint32_t relay_ticks = 0;
    uint32_t relay_timer = 0;
    uint16_t hold_ticks = 0;
    uint16_t hold_timer = 0;
    uint16_t abort_ticks = 0;
    uint16_t abort_timer = 0;
    
    led_ticks = led_persistence_tenths * 1 + 1;
    relay_ticks = timer_seconds * 1 + 1;
    hold_ticks = hold_tenths * 1 + 1;
    abort_ticks = abort_tenths * 1 + 1;
    
    /* Full RT loop. Time: 280*/
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
    
    /*Check for undefined states/LED*/
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
    
    
    /* Recovery from invalid Relay states (Time: 25)
     In: relay_state
     Out: relay_state
     Notes: the code below each conditional should
        never be called under normal conditions
     
     Valid state list:
     states 0,1,8,9,10,12,13
     
     Invalid states:
     All others.
     
     Recovery actions: Go to state 8, shut down device, if button is not pressed
     * state will immediately decay to state 0
     */
    
    /*Set error condition on bit 1 to true*/
    asm("BSF rt_loop@conditionals,1");
    
    /*Clear bit if state is valid*/
    /*State 0*/
    asm("MOVF rt_loop@relay_state,w");
    asm("BTFSC STATUS,2");
    asm("BCF rt_loop@conditionals,1");
    /*State 1*/
    asm("XORLW 1");
    asm("BTFSC STATUS,2");
    asm("BCF rt_loop@conditionals,1");
    /*State 8*/
    asm("XORLW 9");
    asm("BTFSC STATUS,2");
    asm("BCF rt_loop@conditionals,1");
    /*State 9*/
    asm("XORLW 1");
    asm("BTFSC STATUS,2");
    asm("BCF rt_loop@conditionals,1");
    /*State 10*/
    asm("XORLW 3");
    asm("BTFSC STATUS,2");
    asm("BCF rt_loop@conditionals,1");
    /*State 12*/
    asm("XORLW 6");
    asm("BTFSC STATUS,2");
    asm("BCF rt_loop@conditionals,1");
    /*State 13*/
    asm("XORLW 1");
    asm("BTFSC STATUS,2");
    asm("BCF rt_loop@conditionals,1");
    
    /*Recovery code, use conditional bit 1*/
    asm("MOVLW 8");
    asm("BTFSC rt_loop@conditionals,1");
    asm("MOVWF rt_loop@relay_state");
    
    /*Add any code that alerts of failure here, conditioned with bit 0 for 
     * LED states and bit 1 for relay states*/
    
    /* Timers and in-state changes*/
    
    /* Timer decrement/LED (time: 12)
     If timer is on (LED state 1), decrement timer.
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
    
    /*State 1 AND both bytes in zero*/
    asm("IORWF rt_loop@led_timer+1,w");
    asm("BTFSC STATUS,2");
    asm("BSF rt_loop@conditionals,2");
    
    /*Prevent decrement if timer is at zero
      (this should not be necessary, when the timer reaches 0
       the state is changed before the iteration ends...)*/
    asm("BTFSC rt_loop@conditionals,2");
    asm("CLRF rt_loop@conditionals");
    
    /*Decrement timer*/
    asm("BTFSC rt_loop@conditionals,0");
    asm("DECF rt_loop@led_timer");
    asm("BTFSC rt_loop@conditionals,1");
    asm("DECF rt_loop@led_timer+1");
    
    /*Relay Timer decrement (time:28)
     Decrement if bit 0 of the relay state is on
     */
    
    /*Clear conditionals*/
    asm("CLRF rt_loop@conditionals");
    
    /*Check for bit 0 of the relay state and number of timer bytes in zero*/
    asm("MOVF rt_loop@relay_state,w");
    asm("ANDLW 1");
    asm("XORLW 1");
    asm("BTFSC STATUS,2");
    asm("BSF rt_loop@conditionals,0");
    
    asm("IORWF rt_loop@relay_timer,w");
    asm("BTFSC STATUS,2");
    asm("BSF rt_loop@conditionals,1");
       
    asm("IORWF rt_loop@relay_timer+1,w");
    asm("BTFSC STATUS,2");
    asm("BSF rt_loop@conditionals,2");
    
    asm("IORWF rt_loop@relay_timer+2,w");
    asm("BTFSC STATUS,2");
    asm("BSF rt_loop@conditionals,3");
    
    asm("IORWF rt_loop@relay_timer+3,w");
    asm("BTFSC STATUS,2");
    asm("BSF rt_loop@conditionals,4");
    
    /*Prevent decrement if timer is at zero*/
    asm("BTFSC rt_loop@conditionals,4");
    asm("CLRF rt_loop@conditionals");
    
    /*Decrement timer*/
    asm("BTFSC rt_loop@conditionals,0");
    asm("DECF rt_loop@relay_timer");
    asm("BTFSC rt_loop@conditionals,1");
    asm("DECF rt_loop@relay_timer+1");
    asm("BTFSC rt_loop@conditionals,2");
    asm("DECF rt_loop@relay_timer+2");
    asm("BTFSC rt_loop@conditionals,3");
    asm("DECF rt_loop@relay_timer+3");
    
    /*Hold Timer decrement (time:18)
     Decrement if bit 1 of the relay state is on
     */
        
    /*Clear conditionals*/
    asm("CLRF rt_loop@conditionals");
    
    /*Check for bit 1 of the relay state and number of timer bytes in zero*/
    asm("MOVF rt_loop@relay_state,w");
    asm("ANDLW 2");
    asm("XORLW 2");
    asm("BTFSC STATUS,2");
    asm("BSF rt_loop@conditionals,0");
    
    asm("IORWF rt_loop@hold_timer,w");
    asm("BTFSC STATUS,2");
    asm("BSF rt_loop@conditionals,1");
       
    asm("IORWF rt_loop@hold_timer+1,w");
    asm("BTFSC STATUS,2");
    asm("BSF rt_loop@conditionals,2");
    
    /*Prevent decrement if timer is at zero*/
    asm("BTFSC rt_loop@conditionals,2");
    asm("CLRF rt_loop@conditionals");
    
    /*Decrement timer*/
    asm("BTFSC rt_loop@conditionals,0");
    asm("DECF rt_loop@hold_timer");
    asm("BTFSC rt_loop@conditionals,1");
    asm("DECF rt_loop@hold_timer+1");
        
    
    /*Abort Timer decrement (time:18)
     Decrement if bit 2 of the relay state is on
     */
    
    /*Clear conditionals*/
    asm("CLRF rt_loop@conditionals");
    
    /*Check for bit 0 of the relay state and number of timer bytes in zero*/
    asm("MOVF rt_loop@relay_state,w");
    asm("ANDLW 4");
    asm("XORLW 4");
    asm("BTFSC STATUS,2");
    asm("BSF rt_loop@conditionals,0");
    
    asm("IORWF rt_loop@abort_timer,w");
    asm("BTFSC STATUS,2");
    asm("BSF rt_loop@conditionals,1");
       
    asm("IORWF rt_loop@abort_timer+1,w");
    asm("BTFSC STATUS,2");
    asm("BSF rt_loop@conditionals,2");
    
    /*Prevent decrement if timer is at zero*/
    asm("BTFSC rt_loop@conditionals,2");
    asm("CLRF rt_loop@conditionals");
    
    /*Decrement timer*/
    asm("BTFSC rt_loop@conditionals,0");
    asm("DECF rt_loop@abort_timer");
    asm("BTFSC rt_loop@conditionals,1");
    asm("DECF rt_loop@abort_timer+1");
    
    
    /* State changes. Button has precedence*/
    
    /* Button press/LED (time: 3)
     In: gpio_shadow bit 2 (button), led_state
     Out: led_state
     
     Button press changes state to state 2, irrespective of state
     
     Button is pressed when bit is 0, not pressed when bit is 1!!
     */
    asm("MOVLW 2");
    asm("BTFSS rt_loop@gpio_shadow,2");
    asm("MOVWF rt_loop@led_state");
    
    /* Button press/Relay (time: 27)
     In: gpio_shadow, bit 2 (button), relay_state
     out: relay_state
     
     Follow the state machine for details on starting-ending states
     */
    
    /* List of states that take button press events: 0, 1 */
    
    /*Clear conditionals*/
    asm("CLRF rt_loop@conditionals");
    
    /*Test for different states*/
    asm("MOVF rt_loop@relay_state,w");     /*State 0*/
    asm("BTFSC STATUS,2");
    asm("BSF rt_loop@conditionals,0");
    asm("XORLW 1");                          /*State 1*/
    asm("BTFSC STATUS,2");
    asm("BSF rt_loop@conditionals,1");
    
    /*Nullify change if button is not pressed*/
    asm("BTFSC rt_loop@gpio_shadow,2");
    asm("CLRF rt_loop@conditionals");
    
    /*Actions*/
    /*Off (0) -> Starting (10)*/
    asm("MOVLW 10");
    asm("BTFSC rt_loop@conditionals,0");
    asm("MOVWF rt_loop@relay_state");
    asm("MOVF rt_loop@hold_ticks,w");
    asm("BTFSC rt_loop@conditionals,0");
    asm("MOVWF rt_loop@hold_timer");
    asm("MOVF rt_loop@hold_ticks+1,w");
    asm("BTFSC rt_loop@conditionals,0");
    asm("MOVWF rt_loop@hold_timer+1");
    
    /*On (1) -> Aborting (13)*/
    asm("MOVLW 13");
    asm("BTFSC rt_loop@conditionals,1");
    asm("MOVWF rt_loop@relay_state");
    asm("MOVF rt_loop@abort_ticks,w");
    asm("BTFSC rt_loop@conditionals,1");
    asm("MOVWF rt_loop@abort_timer");
    asm("MOVF rt_loop@abort_ticks+1,w");
    asm("BTFSC rt_loop@conditionals,1");
    asm("MOVWF rt_loop@abort_timer+1");
    
    
    /* Button release/LED (time: 16)
     Go to state 1 if state is 2 and button is not pressed, set timer on change
     Else, no action*/
    
    asm("CLRF rt_loop@conditionals");
    asm("MOVF rt_loop@led_state,w");
    asm("BTFSC rt_loop@gpio_shadow,2");
    asm("IORLW 4");
    asm("XORLW 6");
    asm("BTFSC STATUS,2");
    asm("BSF rt_loop@conditionals,0");
    
    asm("MOVLW 1");
    asm("BTFSC rt_loop@conditionals,0");
    asm("MOVWF rt_loop@led_state");
    
    asm("MOVF rt_loop@led_ticks,w");
    asm("BTFSC rt_loop@conditionals,0");
    asm("MOVWF rt_loop@led_timer");
    
    asm("MOVF rt_loop@led_ticks+1,w");
    asm("BTFSC rt_loop@conditionals,0");
    asm("MOVWF rt_loop@led_timer+1");
    
    /*Button release/Relay (time: 24)
     Affects states 9 (starting), 10 (started), 13 (aborting), 8 (aborted) 
     and 12 (FWA). */
    
    /*Clear conditionals*/
    asm("CLRF rt_loop@conditionals");
    
    /*Test for different states*/
    asm("MOVF rt_loop@relay_state,w");
    asm("XORLW 8");                         /*State 8*/
    asm("BTFSC STATUS,2");
    asm("BSF rt_loop@conditionals,0");
    
    asm("XORLW 1");                         /*State 9*/
    asm("BTFSC STATUS,2");
    asm("BSF rt_loop@conditionals,0");
    
    asm("XORLW 3");                         /*State 10*/
    asm("BTFSC STATUS,2");
    asm("BSF rt_loop@conditionals,0");
    
    asm("XORLW 6");                         /*State 12*/
    asm("BTFSC STATUS,2");
    asm("BSF rt_loop@conditionals,0");
    
    asm("XORLW 1");                         /*State 13*/
    asm("BTFSC STATUS,2");
    asm("BSF rt_loop@conditionals,0");
    
    /*Nullify if button is still pressed*/
    asm("BTFSS rt_loop@gpio_shadow,2");
    asm("CLRF rt_loop@conditionals");
    
    /*State changes odd to 1, even to 0*/
    asm("MOVLW 0");
    asm("BTFSC rt_loop@relay_state,0");
    asm("MOVLW 1");
    asm("BTFSC rt_loop@conditionals,0");
    asm("MOVWF rt_loop@relay_state");
    
    /* Relay Timer end (time:7)*/
    asm("CLRW");
    asm("IORWF rt_loop@relay_timer,w");
    asm("IORWF rt_loop@relay_timer+1,w");
    asm("IORWF rt_loop@relay_timer+2,w");
    asm("IORWF rt_loop@relay_timer+3,w");
    asm("BTFSC STATUS,2");
    asm("BCF rt_loop@relay_state,0");
    
    /* Hold Timer end (time: 24)*/
    /*Check for timer running and timer = 0*/
    asm("CLRF rt_loop@conditionals");
    
    asm("MOVF rt_loop@relay_state,w");
    asm("ANDLW 2");
    asm("XORLW 2");
    asm("IORWF rt_loop@hold_timer,w");
    asm("IORWF rt_loop@hold_timer+1,w");
    asm("BTFSC STATUS,2");
    asm("BSF rt_loop@conditionals,0");
    
    /*Stop hold timer, set and start relay timer if true*/
    asm("MOVF rt_loop@relay_state,w");
    asm("XORLW 3");
    asm("BTFSC rt_loop@conditionals,0");
    asm("MOVWF rt_loop@relay_state");
    
    asm("MOVF rt_loop@relay_ticks,w");
    asm("BTFSC rt_loop@conditionals,0");
    asm("MOVWF rt_loop@relay_timer");
    
    asm("MOVF rt_loop@relay_ticks+1,w");
    asm("BTFSC rt_loop@conditionals,0");
    asm("MOVWF rt_loop@relay_timer+1");
    
    asm("MOVF rt_loop@relay_ticks+2,w");
    asm("BTFSC rt_loop@conditionals,0");
    asm("MOVWF rt_loop@relay_timer+2");
    
    asm("MOVF rt_loop@relay_ticks+3,w");
    asm("BTFSC rt_loop@conditionals,0");
    asm("MOVWF rt_loop@relay_timer+3");
    
    /* Abort Timer end (time: 12)*/
    /*Check for timer running and timer = 0*/
    asm("CLRF rt_loop@conditionals");
    
    asm("MOVF rt_loop@relay_state,w");
    asm("ANDLW 4");
    asm("XORLW 4");
    asm("IORWF rt_loop@abort_timer,w");
    asm("IORWF rt_loop@abort_timer+1,w");
    asm("BTFSC STATUS,2");
    asm("BSF rt_loop@conditionals,0");
    
    /*Stop hold timer and stop relay timer if true*/
    asm("MOVF rt_loop@relay_state,w");
    asm("ANDLW 10");
    asm("BTFSC rt_loop@conditionals,0");
    asm("MOVWF rt_loop@relay_state");
    
    /* LED Timer end (time: 5)
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
    
    /*Activate/deactivate Relay (time: 5)
     Matches relay timer bit */
    
    asm("BSF rt_loop@gpio_shadow,5");
    asm("MOVF rt_loop@relay_state,w");
    asm("ANDLW 1");
    asm("BTFSC STATUS,2");
    asm("BCF rt_loop@gpio_shadow,5");
    
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
