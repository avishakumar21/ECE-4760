/*
 * File:        TFT_test_BRL4.c
 * Author:      Bruce Land
 * For use with Sean Carroll's Big Board
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
////////////////////////////////////



// DAC ISR
// A-channel, 1x, active
#define DAC_config_chan_A 0b0011000000000000 // used to display motor
#define DAC_config_chan_B 0b1011000000000000 // used to display angle



#define EnablePullDownB(bits) CNPUBCLR=bits; CNPDBSET=bits;
#define DisablePullDownB(bits) CNPDBCLR=bits;
#define EnablePullUpB(bits) CNPDBCLR=bits; CNPUBSET=bits;
#define DisablePullUpB(bits) CNPUBCLR=bits;

//PORT A
#define EnablePullDownA(bits) CNPUACLR=bits; CNPDASET=bits;
#define DisablePullDownA(bits) CNPDACLR=bits;
#define EnablePullUpA(bits) CNPDACLR=bits; CNPUASET=bits;
#define DisablePullUpA(bits) CNPUACLR=bits;



//== Timer 2 interrupt handler ===========================================
volatile unsigned int DAC_data ;// output value
volatile SpiChannel spiChn = SPI_CHANNEL2 ;	// the SPI channel to use
volatile int spiClkDiv = 2 ; // 20 MHz max speed for this DAC

void __ISR(_TIMER_2_VECTOR, ipl2) Timer2Handler(void)
{
    mT2ClearIntFlag();
    
    // TO READ the two channels
    // adc1 = ReadADC10(0);
    // adc5 = ReadADC10(1); 
    
    // generate  ramp
    // at 100 ksample/sec, 2^12 samples (4096) on one ramp up
    // yields 100000/4096 = 24.4 Hz.
     DAC_data = (DAC_data + 1) & 0xfff ; // for testing
    
    // CS low to start transaction
     mPORTBClearBits(BIT_4); // start transaction
    // test for ready
     while (TxBufFullSPI2());
     // write to spi2
     WriteSPI2(DAC_config_chan_A | DAC_data);
    // test for done
    while (SPI2STATbits.SPIBUSY); // wait for end of transaction
     // CS high
     mPORTBSetBits(BIT_4); // end transaction
}


// === thread structures ============================================
// thread control structs
// note that UART input and output are threads
static struct pt pt_timer ;

// system 1 second interval tick
int sys_time_seconds ;

// === Timer Thread =================================================
// update a 1 second tick counter
static PT_THREAD (protothread_timer(struct pt *pt))
{
    PT_BEGIN(pt);
//     tft_setCursor(0, 0);
//     tft_setTextColor(ILI9340_WHITE);  tft_setTextSize(1);
//     tft_writeString("Time in seconds since boot\n");
//     // set up LED to blink
//     mPORTASetBits(BIT_0 );	//Clear bits to ensure light is off.
//     mPORTASetPinsDigitalOut(BIT_0 );    //Set port as output
//      while(1) {
//        // yield time 1 second
//        PT_YIELD_TIME_msec(1000) ;
//        sys_time_seconds++ ;
//        // toggle the LED on the big board
//        mPORTAToggleBits(BIT_0);
//        // draw sys_time
//        tft_fillRoundRect(0,10, 100, 14, 1, ILI9340_BLACK);// x,y,w,h,radius,color
//        tft_setCursor(0, 10);
//        tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
//        sprintf(buffer,"%d", sys_time_seconds);
//        tft_writeString(buffer);
//        // NEVER exit while
//      } // END WHILE(1)
  PT_END(pt);
} // timer thread

void set_up_ADC(void){
   // configure and enable the ADC
    CloseADC10(); // ensure the ADC is off before setting the configuration

    // define setup parameters for OpenADC10
    // Turn module on | ouput in integer | trigger mode auto | enable autosample
    // ADC_CLK_AUTO -- Internal counter ends sampling and starts conversion (Auto convert)
    // ADC_AUTO_SAMPLING_ON -- Sampling begins immediately after last conversion completes; SAMP bit is automatically set
    // ADC_AUTO_SAMPLING_OFF -- Sampling begins with AcquireADC10();
    #define PARAM1  ADC_FORMAT_INTG16 | ADC_CLK_AUTO | ADC_AUTO_SAMPLING_ON //

    // define setup parameters for OpenADC10
    // ADC ref external  | disable offset test | disable scan mode | do 2 sample | use single buf | alternate mode on
    #define PARAM2  ADC_VREF_AVDD_AVSS | ADC_OFFSET_CAL_DISABLE | ADC_SCAN_OFF | ADC_SAMPLES_PER_INT_2 | ADC_ALT_BUF_OFF | ADC_ALT_INPUT_ON
            //
    // Define setup parameters for OpenADC10
    // use peripherial bus clock | set sample time | set ADC clock divider
    // ADC_CONV_CLK_Tcy2 means divide CLK_PB by 2 (max speed)
    // ADC_SAMPLE_TIME_5 seems to work with a source resistance < 1kohm
    // SLOW it down a little
    #define PARAM3 ADC_CONV_CLK_PB | ADC_SAMPLE_TIME_15 | ADC_CONV_CLK_Tcy //ADC_SAMPLE_TIME_15| ADC_CONV_CLK_Tcy2

    // define setup parameters for OpenADC10
    // set AN11 and  as analog inputs
    #define PARAM4 ENABLE_AN1_ANA | ENABLE_AN5_ANA |ENABLE_AN11_ANA// 

    // define setup parameters for OpenADC10
    // do not assign channels to scan
    #define PARAM5 SKIP_SCAN_ALL //|SKIP_SCAN_AN5 //SKIP_SCAN_AN1 |SKIP_SCAN_AN5  //SKIP_SCAN_ALL

    // // configure to sample AN5 and AN1 on MUX A and B
    SetChanADC10( ADC_CH0_NEG_SAMPLEA_NVREF | ADC_CH0_POS_SAMPLEA_AN1 | ADC_CH0_NEG_SAMPLEB_NVREF | ADC_CH0_POS_SAMPLEB_AN5 |ADC_CH0_NEG_SAMPLEB_NVREF | ADC_CH0_POS_SAMPLEB_AN11 );

    OpenADC10( PARAM1, PARAM2, PARAM3, PARAM4, PARAM5 ); // configure ADC using the parameters defined above

    EnableADC10(); // Enable the ADC

    // TO READ the two channels
    // adc1 = ReadADC10(0);
    // adc5 = ReadADC10(1); 
    // adc11 = ReadADC10(2);
    ///////////////////////////////////////////////////////

}
// === Main  ======================================================
void main(void) {
 //SYSTEMConfigPerformance(PBCLK);
  
  ANSELA = 0; ANSELB = 0; 

 // === Config timer and output compare to make PWM ======== 
    // set up timer2 to generate the wave period every  1 mSec!
    OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_1, 40000);
    // Need ISR to compute PID controller 
    ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_2); 
    mT2ClearIntFlag(); // and clear the interrupt flag 
    
    // set up compare3 for PWM mode 
    OpenOC3(OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE , pwm_on_time, pwm_on_time); // 
    // OC3 is PPS group 4, map to RPB9 (pin 18) 
    PPSOutput(4, RPB9, OC3);
    
    // control CS for DAC
    mPORTBSetPinsDigitalOut(BIT_4);
    mPORTBSetBits(BIT_4);

    // divide Fpb by 2, configure the I/O ports. Not using SS in this example
    // 16 bit transfer CKP=1 CKE=1
    // possibles SPI_OPEN_CKP_HIGH;   SPI_OPEN_SMP_END;  SPI_OPEN_CKE_REV
    // For any given peripherial, you will need to match these
    // clk divider set to 2 for 20 MHz
    SpiChnOpen(SPI_CHANNEL2, SPI_OPEN_ON | SPI_OPEN_MODE16 | SPI_OPEN_MSTEN | SPI_OPEN_CKE_REV , 2);
  // end DAC setup
    
  // === config threads ==========
  // turns OFF UART support and debugger pin, unless defines are set
  PT_setup();
  
    //Two ADC channels (one for angle sensor, one for parameter setting)
    //Perhaps use A5 and A1.
    set_up_ADC();
    
    //RA4 and RA3 push button set up 
    mPORTASetPinsDigitalIn(BIT_3|BIT_4);
    EnablePullDownA( BIT_3 | BIT_4);
    //mPORTAReadBits(BIT_3) or mPORTAReadBits(BIT_4)

    

  // === setup system wide interrupts  ========
  INTEnableSystemMultiVectoredInt();

  // init the threads
  PT_INIT(&pt_timer);
  
  // init the display
  tft_init_hw();
  tft_begin();
  tft_fillScreen(ILI9340_BLACK);
  //240x320 vertical display
  tft_setRotation(0); // Use tft_setRotation(1) for 320x240

  // seed random color
  srand(1);
  
  // round-robin scheduler for threads
  while (1){
      PT_SCHEDULE(protothread_timer(&pt_timer));
      }
  } // main

// === end  ======================================================

