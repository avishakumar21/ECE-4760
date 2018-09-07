/**
 * This is a very small example that shows how to use
 * === OUTPUT COMPARE and INPUT CAPTURE ===
 * The system uses hardware to generate precisely timed
 * pulses, then uses input capture to compare the capture period
 * to the generation period for accuracy
 *
 * There is a capture time print-summary thread
 * There is a one second timer tick thread
 * 
 * -- Pin RB5 and RB9 are output compare outputs
 * -- Pin RB13 is input capture input -- connect this to one of the output compares
 *
 * -- TFT LCD connections explained elsewhere
 * Modified by Bruce Land 
 * Jan 2015
 */

////////////////////////////////////
// clock AND protoThreads configure!
// You MUST check this file!
#include "config.h"
// threading library
#include "pt_cornell_1_2_1.h"

////////////////////////////////////
// graphics libraries
#include "tft_master.h"
#include "tft_gfx.h"
// need for rand function
#include <stdlib.h>
#include <plib.h>
#include <math.h>
////////////////////////////////////

// === thread structures ============================================
// thread control structs

// note that UART input and output are threads
static struct pt pt_print, pt_time ; //, pt_input, pt_output, pt_DMA_output ;

// system 1 second interval tick
int sys_time_seconds ;

//The measured period of the wave
short capture1, last_capture1=0, capture_period=99 ;
//The actual period of the wave
int generate_period=10000 ;

// == Capture 1 ISR ====================================================
// check every capture for consistency
void __ISR(_INPUT_CAPTURE_1_VECTOR, ipl3) C1Handler(void)
{
    
    // read the capture register 
    capture_period = mIC1ReadCapture();
    mIC1ClearIntFlag();
}

// string buffer
      /*static char buffer[128];
       
      tft_setCursor(0, 0);
      tft_setTextColor(ILI9340_WHITE);  tft_setTextSize(2);
      tft_fillRoundRect(0,0, 200, 20, 1, ILI9340_BLACK);// x,y,w,h,radius,color
      sprintf(buffer,"gen period=%d ", capture_period );
      tft_writeString(buffer);
    //  tft_setCursor(0, 20);
    //  tft_writeString("OR RB5 \n");
     // tft_setCursor(0, 40);
      //tft_writeString("to RB13 thru 300ohm\n");
      while(1) {
            // print every 200 mSec
           PT_YIELD_TIME_msec(200) ;
           // erase
            tft_setCursor(0, 80);
            tft_fillRoundRect(0,80, 200, 20, 1, ILI9340_BLACK);// x,y,w,h,radius,color
            // print the periods
             sprintf(buffer,"gen period=%d ", generate_period );
             tft_writeString(buffer);
             
            
             tft_setCursor(0, 100);
               sprintf(buffer,"capture=%d  ", capture_period);
          
      } // END WHILE(1) */

// === Period print Thread ======================================================
// prints the captured period of the generated wave
static PT_THREAD (protothread_print(struct pt *pt))
{
    PT_BEGIN(pt);
    static char buffer[128];
    
     while(1){
            tft_setCursor(0, 0);
            tft_fillCircle(50, 300, 10,ILI9340_RED);
            PT_YIELD_TIME_msec(1000);//creates a toggle with a 1000 millisecond delay
            tft_setCursor(0, 0);
            tft_fillCircle(50, 300, 10,ILI9340_BLACK);
            PT_YIELD_TIME_msec(1000);//creates a toggle with a 1000 millisecond delay
        }
      
      
  PT_END(pt);
} // thread 4

// === One second Thread ======================================================

static PT_THREAD (protothread_time(struct pt *pt))
{
    PT_BEGIN(pt);
    static char buffer[128];
   
   
    while(1){
      mPORTBSetPinsDigitalOut(BIT_3); //Set RB3 as an OUTPUT BIT
      PT_YIELD_TIME_msec(1000); // sets PT YIELD time based 10k ohms
      mPORTBClearBits(BIT_3); // clears the OUTPUT BIT for RB3
      WriteTimer3(0x000);
      mPORTBSetPinsDigitalIn(BIT_3); //Set port as input (pin 7 is RB3)
      PT_YIELD_TIME_msec(200); // sets PT YIELD time based 10k ohms
      float capacitance =  - ((capture_period)) /(200* log((1 - 1.2/ 3.3)));
      if (capacitance > 0.5){
            tft_setCursor(0, 80);
            tft_setTextColor(ILI9340_WHITE);  tft_setTextSize(2);
            tft_fillRoundRect(0,80, 200, 20, 1, ILI9340_BLACK);// x,y,w,h,radius,color   
            sprintf(buffer,"C =%5.1f nF", capacitance );
            tft_writeString(buffer);
            tft_setCursor(0, 20);
     }
      else {
          tft_setCursor(0, 80);
          tft_setTextColor(ILI9340_WHITE);  tft_setTextSize(2);
          tft_fillRoundRect(0,80, 200, 20, 1, ILI9340_BLACK);// x,y,w,h,radius,color   
          sprintf(buffer,"NO CAPACITOR", capacitance );
          tft_writeString(buffer);
          tft_setCursor(0, 20);
      }
      
      PT_YIELD_TIME_msec(200);
    }
  PT_END(pt);
} // 

// === Main  ======================================================

int main(void)
{
  
  // === Config timer and output compares to make pulses ========
  // set up timer2 to generate the wave period
  // count is Zero-based, so subtract one for # of clock ticks
  //OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_1, generate_period-1);

  // set up compare3 for double compare mode
  // first number is the time to clear, second is the time to set the pin
  // in this case, the end of the timer period and 50% of the timer period
   //OpenOC3(OC_ON | OC_TIMER2_SRC | OC_CONTINUE_PULSE , generate_period-1, generate_period>>1); //
  // OC3 is PPS group 4, map to RPB9 (pin 18)
    
    
  

  // set pulse to go high at 1/4 of the timer period and drop again at 1/2 the timer period
  //OpenOC2(OC_ON | OC_TIMER2_SRC | OC_CONTINUE_PULSE, generate_period>>1, generate_period>>2);
  // OC2 is PPS group 2, map to RPB5 (pin 14)
  //PPSOutput(2, RPB5, OC2);

  // === Config timer3 free running ==========================
  // set up timer3 as a souce for input capture
  // and let it overflow for contunuous readings
  
  OpenTimer3(T3_ON | T3_SOURCE_INT | T3_PS_1_4, 0xffff);

  // === set up input capture ================================
  OpenCapture1(  IC_EVERY_RISE_EDGE | IC_INT_1CAPTURE | IC_TIMER3_SRC | IC_ON );
  // turn on the interrupt so that every capture can be recorded
  ConfigIntCapture1(IC_INT_ON | IC_INT_PRIOR_3 | IC_INT_SUB_PRIOR_3 );
  //INTClearFlag(INT_IC1);
   mIC1ClearIntFlag();
  
   // connect PIN 24 to IC1 capture unit
  PPSInput(3, IC1, RPB13);

  // init the display
  tft_init_hw();
  tft_begin();
  tft_fillScreen(ILI9340_BLACK);
  //240x320 vertical display
  tft_setRotation(0); // Use tft_setRotation(1) for 320x240
  tft_setCursor(0, 0);
  
  // === config the uart, DMA, vref, timer5 ISR ===========
  PT_setup();

  // === setup system wide interrupts  ====================
  INTEnableSystemMultiVectoredInt();
  
  // === now the threads ===================================
  // init the threads
  PT_INIT(&pt_print);
  PT_INIT(&pt_time);
  
  
  //set up compare1 
   CMP1Open(CMP_ENABLE | CMP_OUTPUT_ENABLE | CMP1_NEG_INPUT_IVREF);
   PPSOutput(4, RPB9, C1OUT); //pin18
   mPORTBSetPinsDigitalIn(BIT_3); //Set port as input (pin 7 is RB3)

  // schedule the threads
  while(1) {
      PT_SCHEDULE(protothread_print(&pt_print)); //prints red square to screen.
      PT_SCHEDULE(protothread_time(&pt_time));
      
      
  }
} // main
