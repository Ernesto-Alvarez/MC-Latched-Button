/*
 * File:   main.c
 * Author: ealvarez
 *
 * Created on December 7, 2016, 12:11 AM
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

void rt_active(uint16_t,uint8_t,uint8_t,uint8_t);


eeprom uint16_t timer_seconds = 720;
eeprom uint8_t hold_tenths = 50;
eeprom uint8_t abort_tenths = 5;
eeprom uint8_t led_persistence_tenths = 3;

void main(void) {
    init();
    for (;;)
    {
        rt_active(timer_seconds,hold_tenths,abort_tenths,led_persistence_tenths);
    }
}

void rt_active(const uint16_t timer_seconds, const uint8_t hold_tenths, const uint8_t abort_tenths, const uint8_t led_persistence_tenths)
{
    
    uint8_t gpio_shadow = 0;
    uint8_t device_status = 0;
    /*  Bit 0 = Button Status
        Bit 1 = Abort to programming
        Bit 2 = button holding
        Bit 3 = button function
    */
    
    uint8_t debounce_sr = 0;       
    
    uint32_t timer_ticks,timer_counter;
    uint16_t hold_ticks, abort_ticks, hold_timer;
    uint16_t led_ticks,led_counter;

    timer_ticks = 11111 * (uint32_t)timer_seconds;
    hold_ticks = 1111 * hold_tenths;
    abort_ticks = 1111 * abort_tenths;
    led_ticks = 1111 * led_persistence_tenths;
    
    led_counter = 0;
    timer_counter = 0;
    hold_timer = 0;
    
    /*Full RT function Time: 107 */
    asm("start_rt:");
    
    /*  Read GPIO (Time: 2)
        In:     GPIO
        Out:    gpio_shadow
     
     gpio_shadow = GPIO
     */
    
    asm("MOVF GPIO,w");
    asm("MOVWF rt_active@gpio_shadow");
    
    
    /*  Debouncer (time: 4)
        In:     gpio_shadow
        Out:    debounce_sr
     

        debounce_sr = debounce_sr << 2 + gpio_shadow.button

     
    debounce_sr stores the last 8 button presses 
    FF means that the button has been closed for the last 8 ticks
    00 means that the button has been open for the last 8 ticks
    anything else means bouncing is occurring
     */
    
    asm("RLF rt_active@debounce_sr");
    asm("BSF rt_active@debounce_sr,0");
    asm("BTFSS rt_active@gpio_shadow,2");
    asm("BCF rt_active@debounce_sr,0");

    
    /*  Button input (time: 6)
        In:     debounce_sr
        Out:    device_status.button
     
     If (debounce_sr == 0)              
        device_status.button = false;
     If (debounce_sr == 0xff)
        device_status.button = true;
    
     */
    
    asm("MOVF rt_active@debounce_sr,f");   
    asm("BTFSS STATUS,2");                  /*STATUS.2 = ALU ZERO FLAG*/
    asm("BCF rt_active@device_status,0");  
    asm("INCF rt_active@debounce_sr,w");   
    asm("BTFSS STATUS,2");
    asm("BSF rt_active@device_status,0");  
    
    
    
    
    /*  LED Set Timer (time: 10)
        In:     device_status.button, led_ticks
        Out:    gpio_shadow.led, led_counter
    
     if (gpio_shadow.button == true)
     {
        led_timer = led_ticks;
        gpio_shadow.led = true;
     }

    */
        
    asm("BTFSC rt_active@device_status,0");     /*IF button == on*/
    asm("MOVF rt_active@led_ticks,w");
    asm("BTFSC rt_active@device_status,0");     /*THEN counter = led_ticks*/
    asm("MOVWF rt_active@led_counter");
    asm("BTFSC rt_active@device_status,0");
    asm("MOVF rt_active@led_ticks+1,w");
    asm("BTFSC rt_active@device_status,0");
    asm("MOVWF rt_active@led_counter+1");
    asm("BTFSC rt_active@device_status,0");
    asm("BSF rt_active@gpio_shadow,4");         /*AND led = on*/
    
        
    /*  LED Decrement counter (Time: 4)
        In:     led_counter
        Out:    led_counter
     
     led_counter--

     counter keeps decrementing even after reaching zero
     when passing through zero, led is turned off
     button press always turns the led on and sets the times
     guaranteeing that the led will remain on for the specified amount of time
     */
    asm("MOVF rt_active@led_counter,f");        
    asm("BTFSC STATUS,2");                      /*IF ZERO decrement counter high byte*/
    asm("DECF rt_active@led_counter+1,f");
    asm("DECF rt_active@led_counter,f");        
    
    /*  LED Stop (Time: 5)
        In:     led_counter
        Out:    gpio_shadow.led
     
     if (led_counter == 0)
        gpio_shadow.led = false;
     
     */
    asm("CLRW");                            /*IF led_counter == 0*/
    asm("IORWF rt_active@led_counter,w");
    asm("IORWF rt_active@led_counter+1,w");
    asm("BTFSC STATUS,2");                  
    asm("BCF rt_active@gpio_shadow,4");     /*THEN led = off*/
    
        /*Button off (time: 6)
        In:     device_status.button
        Out:    hold_timer, device_status.holding
     
     if (device_status.button == false)
     {
        hold_timer = 0;
        device_status.holding = false;
     }
     
    Clears timers and flags when button is released
     */
    
    asm("BTFSS rt_active@device_status,0");     /*IF button = off*/
    asm("CLRF rt_active@hold_timer");           /*THEN hold_timer = 0*/
    asm("BTFSS rt_active@device_status,0");     
    asm("CLRF rt_active@hold_timer+1");
    asm("BTFSS rt_active@device_status,0");     
    asm("BCF rt_active@device_status,2");       /*AND holding = off */
    
    /* Button push (time: 18)
        In:     device_status.button, device_status.holding
        Out: device_status.holding, device_status.function
     
     if (button == true && holding == false)
     {
        device_status.holding = true
        device_status.function = !gpio_shadow.relay
        if (gpio_shadow.button == false)
             hold_timer = hold_ticks
        else
            hold_timer = abort_ticks
     }
     
     Sets holding flag, captures button status at the time of press and sets
     proper hold timer for the desired function
     */
    
    asm("BTFSS rt_active@device_status,0");     /* IF button = true */
    asm("GOTO push_nop_slide");
    asm("BTFSC rt_active@device_status,2");
    asm("GOTO push_nop_slide_2");               /* AND holding = false*/
    asm("BSF rt_active@device_status,2");       /* THEN holding = true*/
    asm("BCF rt_active@device_status,3");       /* AND function = !gpio_sh.relay*/
    asm("BTFSS rt_active@gpio_shadow,5");
    asm("BSF rt_active@device_status,3");
    asm("MOVF rt_active@hold_ticks,w");       /* set hold_timer*/
    asm("BTFSC rt_active@gpio_shadow,5");
    asm("MOVF rt_active@abort_ticks,w");
    asm("MOVWF rt_active@hold_timer");
    asm("MOVF rt_active@hold_ticks+1,w");
    asm("BTFSC rt_active@gpio_shadow,5");
    asm("MOVF rt_active@abort_ticks+1,w");
    asm("MOVWF rt_active@hold_timer+1");
    asm("GOTO push_end");
    
    asm("push_nop_slide");
    asm("NOP");
    asm("NOP");
    asm("push_nop_slide_2");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("push_end:");
    
    /* Button hold (time: 25)
        In:     device_status.holding, hold_timer, device_status.function
        Out:    hold_timer, gpio_shadow.button
        Precondition: must be run after button off function
     
     if (device_status.holding == true)
     {
     if (hold_timer == 0)
        if (device_status.function == true)
            timer_counter = timer_ticks;
        else
            timer_counter = 0
     else
        hold_timer--
     }
     
     */
    
    
    asm("BTFSS rt_active@device_status,2");     /*IF button being held*/
    asm("GOTO hold_nop_slide");
    asm("CLRW");                                /* THEN IF hold_timer == 0 */
    asm("IORWF rt_active@hold_timer,w");
    asm("IORWF rt_active@hold_timer+1,w");
    asm("BTFSS STATUS,2");
    asm("GOTO hold_dec_timer");
    
    asm("CLRW");                                /* timer_counter = 0 or timer_ticks*/
    asm("BTFSC rt_active@device_status,3");
    asm("MOVF rt_active@timer_ticks,w");
    asm("MOVWF rt_active@timer_counter");
    asm("CLRW");
    asm("BTFSC rt_active@device_status,3");
    asm("MOVF rt_active@timer_ticks+1,w");
    asm("MOVWF rt_active@timer_counter+1");
    asm("CLRW");
    asm("BTFSC rt_active@device_status,3");
    asm("MOVF rt_active@timer_ticks+2,w");
    asm("MOVWF rt_active@timer_counter+2");
    asm("CLRW");
    asm("BTFSC rt_active@device_status,3");
    asm("MOVF rt_active@timer_ticks+3,w");
    asm("MOVWF rt_active@timer_counter+3");
    
    asm("GOTO hold_end");
    
    asm("hold_dec_timer:");                     /* ELSE hold_timer --*/
    asm("MOVF rt_active@hold_timer,f");
    asm("BTFSC STATUS,2");
    asm("DECF rt_active@hold_timer+1,f");
    asm("DECF rt_active@hold_timer,f");
    asm("GOTO hold_dec_end");
    
    asm("hold_nop_slide:");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("hold_dec_end:");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("hold_end:");

    /*Timer control (time: 23)
        In:     timer_counter
        Out:    timer_counter, gpio_shadow.relay
     
     if (timer_counter == 0(
        gpio_shadow.relay = false
     else
     {
        timer_counter--
        gpio_shadow.relay = true
     }
     */
    asm("BSF rt_active@gpio_shadow,5");
    asm("CLRW");
    asm("IORWF rt_active@timer_counter,w");     /*IF timer_counter == 0*/
    asm("IORWF rt_active@timer_counter+1,w");
    asm("IORWF rt_active@timer_counter+2,w");
    asm("IORWF rt_active@timer_counter+3,w");
    asm("BTFSC STATUS,2");                      
    asm("GOTO timer_nop_slide");                
    asm("DECF rt_active@timer_counter,f");      /*ELSE timer_counter--*/
    asm("INCF rt_active@timer_counter,w");      /*(and relay is on)*/
    asm("BTFSS STATUS,2");
    asm("GOTO timer_b1_slide");
    asm("DECF rt_active@timer_counter+1,f");
    asm("INCF rt_active@timer_counter+1,w");
    asm("BTFSS STATUS,2");
    asm("GOTO timer_b2_slide");
    asm("DECF rt_active@timer_counter+2,f");
    asm("INCF rt_active@timer_counter+2,w");
    asm("BTFSS STATUS,2");
    asm("GOTO timer_b3_slide");
    asm("DECF rt_active@timer_counter+3,f");
    asm("GOTO timer_b4_slide");
    asm("timer_nop_slide:");
    asm("BCF rt_active@gpio_shadow,5");         /*THEN relay = off*/
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("timer_b1_slide:");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("timer_b2_slide:");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("timer_b3_slide:");
    asm("NOP");
    asm("NOP");
    asm("timer_b4_slide:");
    
    /* GPIO Commit (time: 2)
        In:     gpio_shadow
        Out:    GPIO
     
     GPIO = gpio_shadow
     
     */

    asm("MOVF rt_active@gpio_shadow,w");
    asm("MOVWF GPIO");
    
    
    /*Loop (time:2)*/
    asm("GOTO start_rt");
    
    asm("end_loop:");
    return;
} 