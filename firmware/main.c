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

/*Bit-banged serial operations, for a future online configuration function*/
uint8_t ttl_recv_byte()
{
    uint8_t serial_data = 0;
    
    //Start bit
    asm("start_bit:");
    asm("BTFSC GPIO,0");
    asm("GOTO start_bit");
    
    asm("MOVLW 5");
    asm("bleed_0:");
    asm("SUBLW 1");
    asm("BTFSS STATUS,2");
    asm("GOTO bleed_0");
    
    asm("NOP");
    asm("NOP");
    asm("NOP");
   
    //Bit 0
    asm("BTFSC GPIO,0");    //Time = 26 +-2
    asm("BSF ttl_recv_byte@serial_data,0");
    
    asm("MOVLW 4");
    asm("bleed_1:");
    asm("SUBLW 1");
    asm("BTFSS STATUS,2");
    asm("GOTO bleed_1");
        
    //Bit 1
    asm("BTFSC GPIO,0");    //Time = 43 +-2
    asm("BSF ttl_recv_byte@serial_data,1");
    
    asm("MOVLW 4");
    asm("bleed_2:");
    asm("SUBLW 1");
    asm("BTFSS STATUS,2");
    asm("GOTO bleed_2");
    
    asm("NOP");
    
    //Bit 2
    asm("BTFSC GPIO,0");    //Time = 61 +-2
    asm("BSF ttl_recv_byte@serial_data,2");
    
    asm("MOVLW 4");
    asm("bleed_3:");
    asm("SUBLW 1");
    asm("BTFSS STATUS,2");
    asm("GOTO bleed_3");
    
    //Bit 3
    asm("BTFSC GPIO,0");    //Time = 78 +-2
    asm("BSF ttl_recv_byte@serial_data,3");
    
    asm("MOVLW 4");
    asm("bleed_4:");
    asm("SUBLW 1");
    asm("BTFSS STATUS,2");
    asm("GOTO bleed_4");
    
    //Bit 4
    asm("BTFSC GPIO,0");    //Time = 95 +-2
    asm("BSF ttl_recv_byte@serial_data,4");
    
    asm("MOVLW 4");
    asm("bleed_5:");
    asm("SUBLW 1");
    asm("BTFSS STATUS,2");
    asm("GOTO bleed_5");
    
    asm("NOP");
    
    //Bit 5
    asm("BTFSC GPIO,0");    //Time = 113 +-2
    asm("BSF ttl_recv_byte@serial_data,5");
    
    asm("MOVLW 4");
    asm("bleed_6:");
    asm("SUBLW 1");
    asm("BTFSS STATUS,2");
    asm("GOTO bleed_6");
    
    //Bit 6
    asm("BTFSC GPIO,0");    //Time = 130 +-2
    asm("BSF ttl_recv_byte@serial_data,6");
    
    asm("MOVLW 4");
    asm("bleed_7:");
    asm("SUBLW 1");
    asm("BTFSS STATUS,2");
    asm("GOTO bleed_7");
    
    //Bit 7
    asm("BTFSC GPIO,0");    //Time = 148 +-2
    asm("BSF ttl_recv_byte@serial_data,7");
    
    return (serial_data);
}

void ttl_send_byte(uint8_t data)
{
    //Start bit
    asm("BCF GPIO,1");
    
    asm("MOVLW 2");
    asm("s_bleed_1:");
    asm("SUBLW 1");
    asm("BTFSS STATUS,2");
    asm("GOTO s_bleed_1");
    
    asm("NOP");
    asm("NOP");
    
    //Bit 0
    asm("MOVF GPIO,w");
    asm("ANDLW 0b11111101");
    asm("RLF ttl_send_byte@data");
    asm("BTFSC STATUS,0");
    asm("IORLW 0b00000010");
    asm("MOVWF GPIO");          //Time = 17
    
    asm("MOVLW 3");
    asm("s_bleed_2:");
    asm("SUBLW 1");
    asm("BTFSS STATUS,2");
    asm("GOTO s_bleed_2");

    
    //Bit 1
    asm("MOVF GPIO,w");
    asm("ANDLW 0b11111101");
    asm("RLF ttl_send_byte@data");
    asm("BTFSC STATUS,0");
    asm("IORLW 0b00000010");
    asm("MOVWF GPIO");          //Time = 35
    
    asm("MOVLW 2");
    asm("s_bleed_3:");
    asm("SUBLW 1");
    asm("BTFSS STATUS,2");
    asm("GOTO s_bleed_3");
    
    asm("NOP");
    asm("NOP");
    asm("NOP");
    
    //Bit 2
    asm("MOVF GPIO,w");
    asm("ANDLW 0b11111101");
    asm("RLF ttl_send_byte@data");
    asm("BTFSC STATUS,0");
    asm("IORLW 0b00000010");
    asm("MOVWF GPIO");          //Time = 52
    
    asm("MOVLW 3");
    asm("s_bleed_4:");
    asm("SUBLW 1");
    asm("BTFSS STATUS,2");
    asm("GOTO s_bleed_4");
    
    //Bit 3
    asm("MOVF GPIO,w");
    asm("ANDLW 0b11111101");
    asm("RLF ttl_send_byte@data");
    asm("BTFSC STATUS,0");
    asm("IORLW 0b00000010");
    asm("MOVWF GPIO");          //Time = 69
    
    asm("MOVLW 3");
    asm("s_bleed_5:");
    asm("SUBLW 1");
    asm("BTFSS STATUS,2");
    asm("GOTO s_bleed_5");
    
    //Bit 4
    asm("MOVF GPIO,w");
    asm("ANDLW 0b11111101");
    asm("RLF ttl_send_byte@data");
    asm("BTFSC STATUS,0");
    asm("IORLW 0b00000010");
    asm("MOVWF GPIO");          //Time = 87
    
    asm("MOVLW 2");
    asm("s_bleed_6:");
    asm("SUBLW 1");
    asm("BTFSS STATUS,2");
    asm("GOTO s_bleed_6");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    
    //Bit 5
    asm("MOVF GPIO,w");
    asm("ANDLW 0b11111101");
    asm("RLF ttl_send_byte@data");
    asm("BTFSC STATUS,0");
    asm("IORLW 0b00000010");
    asm("MOVWF GPIO");          //Time = 104
    
    asm("MOVLW 3");
    asm("s_bleed_7:");
    asm("SUBLW 1");
    asm("BTFSS STATUS,2");
    asm("GOTO s_bleed_7");
    
    //Bit 6
    asm("MOVF GPIO,w");
    asm("ANDLW 0b11111101");
    asm("RLF ttl_send_byte@data");
    asm("BTFSC STATUS,0");
    asm("IORLW 0b00000010");
    asm("MOVWF GPIO");          //Time = 122
    
    asm("MOVLW 2");
    asm("s_bleed_8:");
    asm("SUBLW 1");
    asm("BTFSS STATUS,2");
    asm("GOTO s_bleed_8");
    
    asm("NOP");
    asm("NOP");
    asm("NOP");
    
    //Bit 7
    asm("MOVF GPIO,w");
    asm("ANDLW 0b11111101");
    asm("RLF ttl_send_byte@data");
    asm("BTFSC STATUS,0");
    asm("IORLW 0b00000010");
    asm("MOVWF GPIO");          //Time = 139
    
    asm("MOVLW 4");
    asm("s_bleed_9:");
    asm("SUBLW 1");
    asm("BTFSS STATUS,2");
    asm("GOTO s_bleed_9");
    
    //Stop Bit
    asm("BSF GPIO,1");           //Time = 156
    
    asm("MOVLW 4");
    asm("s_bleed_10:");
    asm("SUBLW 1");
    asm("BTFSS STATUS,2");
    asm("GOTO s_bleed_10");
    
    asm("NOP");
    asm("NOP");
                                //Time = 174
}

eeprom uint16_t timer_seconds = 720;
eeprom uint8_t hold_tenths = 50;
eeprom uint8_t abort_tenths = 2;
eeprom uint8_t led_persistence_tenths = 1;

void main(void) {
    init();
    for (;;)
    {
        rt_active(timer_seconds,hold_tenths,abort_tenths,led_persistence_tenths);
    }
}

void rt_active(const uint16_t timer_seconds, const uint8_t hold_tenths, const uint8_t abort_tenths, const uint8_t led_persistence_tenths)
{
    uint8_t conditionals = 0;
    uint8_t gpio_shadow = 0;
    uint8_t device_status = 0;
    /*  Bit 0 = Button Status
        Bit 1 = Abort to programming
        Bit 2 = button holding
        Bit 3 = button function
    */
    
    /*This is the reason for the initial LED blink*/
    /*Cannot be set to FF because then it will remain on for 64k ticks*/
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
    
    /*Full RT function Time: 119 */
    asm("start_rt:");
    
    /*  Read GPIO (Time: 2)
        In:     GPIO
        Out:    gpio_shadow
     
     gpio_shadow = GPIO
     */
    
    asm("MOVF GPIO,w");
    asm("MOVWF rt_active@gpio_shadow");
    
 
    /*Button input (time: 3)
     In: GPIO_shadow
     Out: device_status
     Inverts the status of the hardware button, causing device_status.button to
     be 1 when the button is pressed and 0 when is not
     We got rid of the broken debouncer replacing it with nothing. Debouncing
     is not a problem with the hold timers, as that works as a debouncer as well.
     */
    
    asm("BCF rt_active@device_status,0");    
    asm("BTFSS rt_active@gpio_shadow,2");
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
    
        /*Button off (time: 2)
        In:     device_status.button
        Out:    hold_timer, device_status.holding
     
     if (device_status.button == false)
     {
        hold_timer = 0;
        device_status.holding = false;
     }
     
    Clears timers and flags when button is released
       
    !!!!!! I think we could skip clearing the hold_timer
    !!!!!! The timer will keep counting, but since holding is off, and 
    !!!!!! button press resets the timer, i doesn't matter WHERE it is counting
     */
    
//    asm("BTFSS rt_active@device_status,0");     /*IF button = off*/
//    asm("CLRF rt_active@hold_timer");           /*THEN hold_timer = 0*/
//    asm("BTFSS rt_active@device_status,0");     
//    asm("CLRF rt_active@hold_timer+1");
    asm("BTFSS rt_active@device_status,0");     
    asm("BCF rt_active@device_status,2");       /*AND holding = off */
    
    /* Button push (time: 33)
        In:     device_status.button, device_status.holding
        Out: device_status.holding, device_status.function
        Uses: conditionals
     
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
    
    /*Set conditionals*/
       
    asm("CLRF rt_active@conditionals");
    asm("MOVF rt_active@device_status,w");  /*IF button==True*/
    asm("ANDLW 13");                        /*AND holding==False*/
    asm("XORLW 1");                         /*AND function==False*/
    asm("BTFSC STATUS,2");
    asm("BSF rt_active@conditionals,0");    
    asm("XORLW 8");                         /* SAME BUT function == True*/
    asm("BTFSC STATUS,2");
    asm("BSF rt_active@conditionals,1");
    
    /*IF BUTTON == True AND HOLDING == False AND FUNCTION == False*/
    asm("BTFSC rt_active@conditionals,0");
    asm("BSF rt_active@device_status,2");   /*THEN HOLDING=True*/
    asm("BTFSC rt_active@conditionals,0");
    asm("MOVF rt_active@hold_ticks,w");     /*  hold_timer = hold_ticks */
    asm("BTFSC rt_active@conditionals,0");
    asm("MOVWF rt_active@hold_timer");
    asm("BTFSC rt_active@conditionals,0");
    asm("MOVF rt_active@hold_ticks+1,w");
    asm("BTFSC rt_active@conditionals,0");
    asm("MOVWF rt_active@hold_timer+1");
    asm("BTFSC rt_active@conditionals,0");
    asm("BSF rt_active@device_status,3");   /*  function = true */
    
    
    /*IF BUTTON == True AND HOLDING == False AND FUNCTION == True*/
    asm("BTFSC rt_active@conditionals,1");
    asm("BSF rt_active@device_status,2");   /*THEN HOLDING=True*/
    asm("BTFSC rt_active@conditionals,1");
    asm("MOVF rt_active@abort_ticks,w");     /*  hold_timer = abort_ticks */
    asm("BTFSC rt_active@conditionals,1");
    asm("MOVWF rt_active@hold_timer");
    asm("BTFSC rt_active@conditionals,1");
    asm("MOVF rt_active@abort_ticks+1,w");
    asm("BTFSC rt_active@conditionals,1");
    asm("MOVWF rt_active@hold_timer+1");
    asm("BTFSC rt_active@conditionals,1");
    asm("BCF rt_active@device_status,3");   /*  function = false */
       
    
    
    
    
    
    
    
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