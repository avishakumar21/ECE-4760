/*
 * File:        TFT_test_BRL4.c
 * Author:      Bruce Land
 * Adapted from:
 *              main.c by
 * Author:      Syed Tahmid Mahbub
 * Target PIC:  PIC32MX250F128B
 */

////////////////////////////////////
// clock AND protoThreads configure!
// You MUST check this file!
#include "config.h"
// threading library
#include "pt_cornell_1_2_3.h"

////////////////////////////////////
// graphics libraries
#include "tft_master.h"
#include "tft_gfx.h"
// need for rand function
#include <stdlib.h>
#include "math.h"
////////////////////////////////////


/* Demo code for interfacing TFT (ILI9340 controller) to PIC32
 * The library has been modified from a similar Adafruit library
 */
// Adafruit data:
/***************************************************
  This is an example sketch for the Adafruit 2.2" SPI display.
  This library works with the Adafruit 2.2" TFT Breakout w/SD card
  ----> http://www.adafruit.com/products/1480

  Check out the links above for our tutorials and wiring diagrams
  These displays use SPI to communicate, 4 or 5 pins are required to
  interface (RST is optional)
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/
#define DAC_config_chan_A 0b0011000000000000

// string buffer
char buffer[60];

// === thread structures ============================================
// thread control structs
// note that UART input and output are threads
static struct pt pt_timer, pt_color, pt_anim ;

// system 1 second interval tick
int sys_time_seconds ;

//constructs default sine table 
volatile short sin_table[256];

// === Timer Thread =================================================
// update a 1 second tick counter
static PT_THREAD (protothread_timer(struct pt *pt))
{
    PT_BEGIN(pt);
     tft_setCursor(0, 0);
     tft_setTextColor(ILI9340_WHITE);  tft_setTextSize(1);
//     tft_writeString("Time in seconds since boot\n");
      while(1) {
        // yield time 1 second
        PT_YIELD_TIME_msec(1000) ;
        sys_time_seconds++ ;
        
        // draw sys_time
        tft_fillRoundRect(0,10, 100, 14, 1, ILI9340_BLACK);// x,y,w,h,radius,color
        tft_setCursor(0, 10);
        tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
        sprintf(buffer,"%d", sys_time_seconds);
        tft_writeString(buffer);
        // NEVER exit while
      } // END WHILE(1)
  PT_END(pt);
} // timer thread


// ===  Ball Animation Thread =============================================
// update a 1 second tick counter
static _Accum xc=(_Accum)(100), yc=(_Accum)(100), vxc=(_Accum)(8), vyc=(_Accum)(0);
static _Accum drag = (_Accum)(.0001);
volatile int current_poten, past_poten=0;// used to save the current and the past_potentiometer values
static float new_poten = 0;

static PT_THREAD (protothread_anim(struct pt *pt))
{
    PT_BEGIN(pt);
    
      while(1) {
        // yield time 1 second
        PT_YIELD_TIME_msec(32);
        
        // read the ADC AN11 
        // read the first buffer position
        new_poten = ReadADC10(0);
        current_poten = (((new_poten * 214))/1023  + 3); // reads the new potentiometer location
        //updating the paddle drawing
        if (current_poten != past_poten){  
            //avoids un-needed redraws of the paddle 
            tft_reDrawPaddle (past_poten,current_poten,20);
            past_poten = current_poten;
        }
//        tft_fillRoundRect(0,10, 60, 14, 1, ILI9340_BLACK);// x,y,w,h,radius,color
//        tft_setCursor(0, 10);
//        tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
//        sprintf(buffer,"%d", current_poten);
//        tft_writeString(buffer);
        // erase disk
         tft_drawSmallCircle((int)(xc), (int)(yc), ILI9340_BLACK); //x, y, radius, color
        
         // compute new velocities
         vyc = vyc - (vyc*drag) ;
         vxc = vxc - (vxc*drag);
         
         xc = xc + vxc;
         yc = yc + vyc;
         
         if (yc<(_Accum)(4)){ vyc = -vyc;  yc = (_Accum)(8) -yc;}
         else if ( yc>(_Accum)(236)) { vyc = -vyc;  yc = (_Accum)(472)- yc;}
         
         if (xc<(_Accum)(4)){ vxc = -vxc;  xc = (_Accum)(8) - xc;}
         else if (xc>(_Accum)(316)){ xc = (_Accum)(632)-xc; vxc = -vxc;}
         else if ( (xc> (_Accum)(76) && xc< (_Accum)(86))&&(yc < (_Accum)(60) || yc> (_Accum)(180))){
             vxc = -vxc;
             xc = ( (xc +vxc )< (_Accum)(76) )?((_Accum)(152)-xc):((_Accum)(172)-xc);
             
         }
         else if (xc< (_Accum)(45) && xc> (_Accum)(37) && 
                 yc > (_Accum)( (past_poten - 5)) && yc < (_Accum)((past_poten +25))){
             //
             vxc = -vxc;
             xc = ((_Accum)(90)-xc);
         }
         //  draw disk
         tft_drawSmallCircle((int)(xc), (int)(yc), ILI9340_RED); //x, y, radius, color
        
      } // END WHILE(1)
  PT_END(pt);
} // animation thread

// === Main  ======================================================
void main(void) {
 //SYSTEMConfigPerformance(PBCLK);
       // the ADC ///////////////////////////////////////
   // configure and enable the ADC
	CloseADC10();	// ensure the ADC is off before setting the configuration
    
    // define setup parameters for OpenADC10
	// Turn module on | ouput in integer | trigger mode auto | enable autosample
        // ADC_CLK_AUTO -- Internal counter ends sampling and starts conversion (Auto convert)
        // ADC_AUTO_SAMPLING_ON -- Sampling begins immediately after last conversion completes; SAMP bit is automatically set
        // ADC_AUTO_SAMPLING_OFF -- Sampling begins with AcquireADC10();
    #define PARAM1  ADC_FORMAT_INTG16 | ADC_CLK_AUTO | ADC_AUTO_SAMPLING_ON //

	// define setup parameters for OpenADC10
	// ADC ref external  | disable offset test | disable scan mode | do 1 sample | use single buf | alternate mode off
	#define PARAM2  ADC_VREF_AVDD_AVSS | ADC_OFFSET_CAL_DISABLE | ADC_SCAN_OFF | ADC_SAMPLES_PER_INT_1 | ADC_ALT_BUF_OFF | ADC_ALT_INPUT_OFF
        //
	// Define setup parameters for OpenADC10
    // use peripherial bus clock | set sample time | set ADC clock divider
    // ADC_CONV_CLK_Tcy2 means divide CLK_PB by 2 (max speed)
    // ADC_SAMPLE_TIME_5 seems to work with a source resistance < 1kohm
    #define PARAM3 ADC_CONV_CLK_PB | ADC_SAMPLE_TIME_5 | ADC_CONV_CLK_Tcy2 //ADC_SAMPLE_TIME_15| ADC_CONV_CLK_Tcy2

	// define setup parameters for OpenADC10
	// set AN11 and  as analog inputs
	#define PARAM4	ENABLE_AN11_ANA // pin 24

	// define setup parameters for OpenADC10
	// do not assign channels to scan
	#define PARAM5	SKIP_SCAN_ALL

	// use ground as neg ref for A | use AN11 for input A     
	// configure to sample AN11 
	SetChanADC10( ADC_CH0_NEG_SAMPLEA_NVREF | ADC_CH0_POS_SAMPLEA_AN11 ); // configure to sample AN11 
	OpenADC10( PARAM1, PARAM2, PARAM3, PARAM4, PARAM5 ); // configure ADC using the parameters defined above

	EnableADC10(); // Enable the ADC
  ///////////////////////////////////////////////////////
  ANSELA = 0; ANSELB = 0; 

  // === config threads ==========
  // turns OFF UART support and debugger pin, unless defines are set
  PT_setup();

  // === setup system wide interrupts  ========
  INTEnableSystemMultiVectoredInt();
  
  //Sine table 
   int sine_table_size = 256; int i;
   for (i = 0; i < sine_table_size; i++){
         sin_table[i] =  ( DAC_config_chan_A|((short)(2047*sin((float)i*6.283/(float)sine_table_size))));
    }
       #define	SYS_FREQ 40000000
        int timer_limit ;

        // calibrate the internal clock
        SYSKEY = 0x0; // ensure OSCCON is locked
        SYSKEY = 0xAA996655; // Write Key1 to SYSKEY
        SYSKEY = 0x556699AA; // Write Key2 to SYSKEY
        // OSCCON is now unlocked
        // make the desired change
        OSCTUN = 31; // tune the rc oscillator
        // Relock the SYSKEY
        SYSKEY = 0x0; // Write any value other than Key1 or Key2
        // OSCCON is relocked

       // Configure the device for maximum performance but do not change the PBDIV
	// Given the options, this function will change the flash wait states, RAM
	// wait state and enable prefetch cache but will not change the PBDIV.
	// The PBDIV value is already set via the pragma FPBDIV option above..
	SYSTEMConfig(SYS_FREQ, SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE);
        
        // frequency settable to better than 2% relative accuracy at low frequency
        // frequency settable to 5% accuracy at highest frequency
        // Useful frequency range 10 Hz to 100KHz
        // >40 db attenuation of  next highest amplitude frequency component below 100 kHz
        // >35 db above 100 KHz
        #define F_OUT 44000
  
         sine_table_size = 256 ;
         timer_limit = SYS_FREQ/(sine_table_size*F_OUT) ;
    
        //Enabling timer and setting up SPI audio file transfer 
        // 4400 is 44 ksamples/sec
        OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_1, timer_limit);
        mT2ClearIntFlag(); // and clear the interrupt flag
        //    // Begin DMA SET UP
         #define dmaChn 0
         // We enable the AUTO option, we'll keep repeating the sam transfer over and over.
         DmaChnOpen(dmaChn, 0, DMA_OPEN_AUTO);

         // set the transfer parameters: source & destination address, source & destination size, number of bytes per event
         // Setting the last parameter to one makes the DMA output one byte/interrupt
          DmaChnSetTxfer(dmaChn, sin_table, (void*)&SPI2BUF, 2*sine_table_size, 2, 2);

         // set the transfer event control: what event is to start the DMA transfer
         // In this case, timer2
         DmaChnSetEventControl(dmaChn, DMA_EV_START_IRQ(_TIMER_2_IRQ));

      //   DmaChnEnable(dmaChn);

    //end DMA SET UP 
  

        // SCK2 is pin 26 
        // SDO2 (MOSI) is in PPS output group 2, could be connected to RB5 which is pin 14
        PPSOutput(2, RPB5, SDO2);

 
        SpiChnOpen(SPI_CHANNEL2, 
                SPI_OPEN_ON | SPI_OPEN_MODE16 | SPI_OPEN_MSTEN | SPI_OPEN_CKE_REV | SPICON_FRMEN | SPICON_FRMPOL,
                2);
    // SS1 to RPB4 for FRAMED SPI
        PPSOutput(4, RPB10, SS2);

  // init the threads
  PT_INIT(&pt_anim);

  // init the display
  tft_init_hw();
  tft_begin();
  tft_fillScreen(ILI9340_BLACK);
  //240x320 vertical display
  tft_setRotation(1); // Use tft_setRotation(1) for 240x320
  
  // drawing the borders
    tft_fillRect (0,0,2,240,ILI9340_WHITE);

    tft_fillRect (318,0,2,240,ILI9340_WHITE);

    tft_fillRect (0,0,320,2,ILI9340_WHITE);

    tft_fillRect (0,238,320,2,ILI9340_WHITE);

    // drawing the ditches 
    tft_fillRect (80,0,2,60,ILI9340_WHITE);
    tft_fillRect (80,180,2,60,ILI9340_WHITE);
 
 
  new_poten = ReadADC10(0); // this is going to be used to seed the random number
  
  past_poten = (((new_poten * 214))/1023  + 3); // reads the new potentiometer location
  
  //drawing the paddle
  tft_fillRect(40, (past_poten), 2, 20 , ILI9340_WHITE);
  
        
  
   
  // round-robin scheduler for threads
  while (1){
      PT_SCHEDULE(protothread_anim(&pt_anim));
      }
  } // main

// === end  ======================================================
