/*********************************************************************
 *  DDS demo code
 *  sine synth to SPI to  MCP4822 dual channel 12-bit DAC
 *********************************************************************
 * Bruce Land Cornell University
 * Sept 2017
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
/* ====== MCP4822 control word ==============
bit 15 A/B: DACA or DACB Selection bit
1 = Write to DACB
0 = Write to DACA
bit 14 : Don't Care
bit 13 GA: Output Gain Selection bit
1 = 1x (VOUT = VREF * D/4096)
0 = 2x (VOUT = 2 * VREF * D/4096), where internal VREF = 2.048V.
bit 12 SHDN: Output Shutdown Control bit
1 = Active mode operation. VOUT is available. 
0 = Shutdown the selected DAC channel. Analog output is not available at the channel that was shut down.
bit 11-0 D11:D0: DAC Input Data bits. 
*/
#define EnablePullDownB(bits) CNPUBCLR=bits; CNPDBSET=bits;
#define DisablePullDownB(bits) CNPDBCLR=bits;
#define EnablePullUpB(bits) CNPDBCLR=bits; CNPUBSET=bits;
#define DisablePullUpB(bits) CNPUBCLR=bits;
//PORT A
#define EnablePullDownA(bits) CNPUACLR=bits; CNPDASET=bits;
#define DisablePullDownA(bits) CNPDACLR=bits;
#define EnablePullUpA(bits) CNPDACLR=bits; CNPUASET=bits;
#define DisablePullUpA(bits) CNPUACLR=bits;

// A-channel, 1x, active
#define DAC_config_chan_A 0b0011000000000000
#define DAC_config_chan_B 0b1011000000000000
#define Fs 23000.0
#define two32 4294967296.0 // 2^32 


////////////////////////////////////
// clock AND protoThreads configure!
// You MUST check this file!
#include "config.h"
// threading library
#include "pt_cornell_1_2_3.h"
// for sine
#include <math.h>
volatile SpiChannel spiChn = SPI_CHANNEL2 ;	// the SPI channel to use
// for 60 MHz PB clock use divide-by-3
volatile int spiClkDiv = 2 ; // 20 MHz DAC clock

////////////////////////////////////
// graphics libraries
// SPI channel 1 connections to TFT
#include "tft_master.h"
#include "tft_gfx.h"
////////////////////////////////////
// some precise, fixed, short delays
// to use for extending pulse durations on the keypad
// if behavior is erratic
#define NOP asm("nop");
// 1/2 microsec
#define wait20 NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;NOP;
// one microsec
#define wait40 wait20;wait20;
////////////////////////////////////

// === thread structures ============================================
// thread control structs
// note that UART input and output are threads
static struct pt pt_param,pt_key;

// DDS sine table
#define sine_table_size 256
volatile int sin_table[sine_table_size];

//== Timer 2 interrupt handler ===========================================
// actual scaled DAC 
volatile  int DAC_data, DAC_data1;
// the DDS units:
volatile unsigned int phase_accum_main, phase_incr_main=0 ;//

// the DDS units:
volatile unsigned int phase_accum_main1, phase_incr_main1=0 ;//



// profiling of ISR
volatile int isr_enter_time, isr_exit_time;
// demo float to test isr timing
volatile float test_isr_float = 0.5 ;
volatile int test_isr_int = 2 ;

volatile int five_msec_delay=1; //used to signal if it's ramp up or ramp down

volatile int number_index,ramp,toggle =0; // used to figure out which signal sound needs to be generated

//=============================

volatile int PushState,  prior_press,counter,sound_index, t_index=0;

static int key_table[12];

static float freq1[] = {1209, 1336, 1477, 1209, 1336, 1477, 1209, 1336, 1477, 1336} ;
static float freq2[] = {697, 697, 697, 770, 770, 770, 852, 852, 852, 941} ;

//****************************************************/
// print a line on the TFT
// string buffer
char buffer[60];
 
void __ISR(_TIMER_2_VECTOR, ipl2) Timer2Handler(void)
{
    // 74 cycles to get to this point from timer event
    isr_enter_time = ReadTimer2() ; // - isr_time;
    
    mT2ClearIntFlag();
    

    if (PushState == 2 && toggle == 0){
        phase_accum_main += phase_incr_main  ;
        phase_accum_main1 += phase_incr_main1 ;
        counter = 0;
        
         //sine_index = phase_accum_main>>24 ;
        DAC_data = sin_table[phase_accum_main>>24];


        //sine_index = phase_accum_main>>24 ;
        DAC_data1 = sin_table[phase_accum_main1>>24];



        // === Channel A =============
        // CS low to start transaction
         mPORTBClearBits(BIT_4); // start transaction
        // test for ready
         //while (TxBufFullSPI2());
        // write to spi2 

        WriteSPI2( DAC_config_chan_A | ( ((DAC_data +DAC_data1)>>1)+ 2048));
        
    }
    else if (toggle ==1 && number_index <t_index){
        counter =counter + 1;
        // 5 msec time is  = 262144  which is 2^18

         if (counter >= 484){
                
              // five_msec_delay =1;
               number_index =1+ number_index;
               if (number_index <t_index){
               phase_accum_main1, phase_incr_main1= freq2[key_table[number_index]]; 
               phase_accum_main, phase_incr_main = freq1[key_table[number_index]];
               }
               else {
                   toggle = 0;
                   number_index = 0;
                   phase_accum_main1, phase_incr_main1= 0; 
                   phase_accum_main, phase_incr_main = 0;
               }
               counter=0;
               ramp = 0;
         }
         else {
             //ramp logic
             if (counter<35){
             }
             else if (counter < 207){
             
             }
             else if (counter < 242){
             }
             else {
             
             }
             
         }
         phase_accum_main += phase_incr_main  ;
         phase_accum_main1 += phase_incr_main1 ;
        
         //sine_index = phase_accum_main>>24 ;
        DAC_data = sin_table[phase_accum_main>>24];


        //sine_index = phase_accum_main1>>24 ;
        DAC_data1 = sin_table[phase_accum_main1>>24];


         mPORTBClearBits(BIT_4); // start transaction
        // test for ready
         //while (TxBufFullSPI2());
        // write to spi2 

        WriteSPI2( DAC_config_chan_A | ( ((DAC_data +DAC_data1)>>1)+ 2048));
        
    
    
    }
    else {
        // might be the case where playback finished and no sound is coming out
        toggle = 0;
        number_index = 0;
        
        mPORTBClearBits(BIT_4); // start transaction
        WriteSPI2( DAC_config_chan_A | ( 2048));
    
    
    }
        
 
    
//    // main DDS phase
//    //phase_accum_main += phase_incr_main  ;
//    
//    //sine_index = phase_accum_main>>24 ;
//    DAC_data = sin_table[phase_accum_main>>24];
//    
//    
//    //sine_index = phase_accum_main>>24 ;
//    DAC_data1 = sin_table[phase_accum_main1>>24];
//    
//    // The following section is purely for testing mult speed!
//    // Optimization set at level 1
//    // comment out one or both to see effect on ISR time!
//    // With neither: ISR total time is 123 cycles (enter, compute, exit)
//    // With float mult, ISR total time is 269  cycles 
//    // -- entry time doubles because more reg need to be saved
//    // With int mult and shift, ISR total time is 128  cycles 
//    // now the test multiply
//    // comment out to see float mult time
//    //DAC_data = (int)(DAC_data*test_isr_float);
//    //DAC_data = (int)(DAC_data*test_isr_float);
//    // comment out to see int shift/multiply time   
//    //    DAC_data = (DAC_data*test_isr_int)>>1;
//    //    DAC_data1 = (DAC_data1*test_isr_int)>>1;
//    // end test section
//    
//    // === Channel A =============
//    // CS low to start transaction
//     mPORTBClearBits(BIT_4); // start transaction
//    // test for ready
//     //while (TxBufFullSPI2());
//    // write to spi2 
//     
//     //sprintf(buffer,"%d", ( ((DAC_data +DAC_data1)>>1)+ 2048));
//    WriteSPI2( DAC_config_chan_A | ( ((DAC_data +DAC_data1)>>1)+ 2048));
    while (SPI2STATbits.SPIBUSY); // wait for end of transaction
     // CS high
    mPORTBSetBits(BIT_4); // end transaction
    isr_exit_time = ReadTimer2() ; // - isr_time;
     
} // end ISR TIMER2



void printLine(int line_number, char* print_buffer, short text_color, short back_color){
    // line number 0 to 31 
    /// !!! assumes tft_setRotation(0);
    // print_buffer is the string to print
    int v_pos;
    v_pos = line_number * 10 ;
    // erase the pixels
    tft_fillRoundRect(0, v_pos, 239, 8, 1, back_color);// x,y,w,h,radius,color
    tft_setTextColor(text_color); 
    tft_setCursor(0, v_pos);
    tft_setTextSize(1);
    tft_writeString(print_buffer);
}

void printLine2(int line_number, char* print_buffer, short text_color, short back_color){
    // line number 0 to 31 
    /// !!! assumes tft_setRotation(0);
    // print_buffer is the string to print
    int v_pos;
    v_pos = line_number * 20 ;
    // erase the pixels
    tft_fillRoundRect(0, v_pos, 319, 16, 1, back_color);// x,y,w,h,radius,color
    tft_setTextColor(text_color); 
    tft_setCursor(0, v_pos);
    tft_setTextSize(2);
    tft_writeString(print_buffer);
}

// used to increment FSM and create the table

static void resetTable(){
     t_index = 0;
}

static void addTable(){
    if (t_index<12) {
        key_table[t_index] = prior_press;
        t_index ++;
    }
}

static PT_THREAD (protothread_debounce(struct pt *pt)){
     PT_BEGIN(pt);
    
     
  
     static int keypad, i, pattern;
    
    // order is 0 thru 9 then * ==10 and # ==11
    // no press = -1
    // table is decoded to natural digit order (except for * and #)
    // 0x80 for col 1 ; 0x100 for col 2 ; 0x200 for col 3
    // 0x01 for row 1 ; 0x02 for row 2; etc
    static int keytable[12]={0x108, 0x81, 0x101, 0x201, 0x82, 0x102, 0x202, 0x84, 0x104, 0x204, 0x88, 0x208};
    // init the keypad pins A0-A3 and B7-B9
    // PortA ports as digital outputs
    mPORTASetPinsDigitalOut(BIT_0 | BIT_1 | BIT_2 | BIT_3);    //Set port as output
    // PortB as inputs
    mPORTBSetPinsDigitalIn(BIT_7 | BIT_8 | BIT_9);    //Set port as input
    // and turn on pull-down on inputs
    EnablePullDownB( BIT_7 | BIT_8 | BIT_9);
    
    
    
     
     while(1){
         
           // yield time
        PT_YIELD_TIME_msec(30);
        
        // read each row sequentially
        mPORTAClearBits(BIT_0 | BIT_1 | BIT_2 | BIT_3);
        pattern = 1; mPORTASetBits(pattern);
   
        for (i=0; i<4; i++) {
            wait40 ;
            keypad  = mPORTBReadBits(BIT_7 | BIT_8 | BIT_9);
            if(keypad!=0) {keypad |= pattern ; break;}
            mPORTAClearBits(pattern);
            pattern <<= 1;
            mPORTASetBits(pattern);
        }
        
        // search for keycode
        if (keypad > 0){ // then button is pushed
            for (i=0; i<12; i++){
                if (keytable[i]==keypad) break;
            }
            // if invalid, two button push, set to -1
            if (i==12) i=-1;
        }
        else i = -1; // no button pushed
        
         
        
        
        switch (PushState) {

               case 0: 
                  if (i != -1) {PushState=1; prior_press = i;}
                  else {PushState=0;}
                  break;

               case 1:
                  if (prior_press == i) {
                     PushState=2;
                     // resets the table if they press star
                     if (i ==10){ 
                         resetTable();
                         phase_accum_main1, phase_incr_main1= 0;
                       phase_accum_main, phase_incr_main= 0;
                         
                     } 
                      //sets the playback value to true
                     else if (i == 11 && toggle == 0){
                        toggle = 1;  //sets playback to true
                        number_index = 0;
                        if (number_index<t_index){
                            phase_accum_main1, phase_incr_main1= freq2[key_table[0]];
                            phase_accum_main, phase_incr_main= freq1[key_table[0]];
                        }
                        else{
                             phase_accum_main1, phase_incr_main1= 0;
                             phase_accum_main, phase_incr_main= 0;
                             
                        }
                     }
                    
                     else if (toggle == 0 ) {
                    //adds the value to table for playback assuming playback isn't happening 
                       phase_accum_main1, phase_incr_main1= freq2[i];
                       phase_accum_main, phase_incr_main= freq1[i];
                         
                      addTable();
                     
                     } 

                  }
                  else{ PushState=0;}
                  break;

               case 2:  
                  if (prior_press == i ) PushState=2; 
                  else PushState=3;    
                  break;

               case 3:
                  if (prior_press == i) PushState=2; 
                  else {PushState=0;}    
                  break;
               
            default :
                if (i != -1) {PushState=1; prior_press = i;}
                  else{ PushState=0;}
                  break;
            } // end case
        
         if (PushState == 2){

                tft_fillRoundRect(30,100, 100, 14, 1, ILI9340_BLACK);// x,y,w,h,radius,color
                tft_setCursor(30, 100);
                tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
                 sprintf(buffer,"%d", i);
                 if (i==10)sprintf(buffer,"*");
                 if (i==11)sprintf(buffer,"#");
                tft_writeString(buffer);
             
             }
        
                 
 
        
      } // END WHILE(1)
  PT_END(pt);
} //key press thread



// === Main  ======================================================
// set up UART, timer2, threads
// then schedule them as fast as possible

int main(void)
{
    
  /// timer interrupt //////////////////////////
    // Set up timer2 on,  interrupts, internal clock, prescalar 1, toggle rate
    //(1740 sample per second at 20k Hz) #128 ramp up counter
    
    OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_1, 1740);
    // set up the timer interrupt with a priority of 2
    ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_2);
    mT2ClearIntFlag(); // and clear the interrupt flag

    /// SPI setup //////////////////////////////////////////
    // SCK2 is pin 26 
    // SDO2 is in PPS output group 2, could be connected to RB5 which is pin 14
    PPSOutput(2, RPB5, SDO2);
    // control CS for DAC
    mPORTBSetPinsDigitalOut(BIT_4);
    mPORTBSetBits(BIT_4);
    // divide Fpb by 2, configure the I/O ports. Not using SS in this example
    // 16 bit transfer CKP=1 CKE=1
    // possibles SPI_OPEN_CKP_HIGH;   SPI_OPEN_SMP_END;  SPI_OPEN_CKE_REV
    // For any given peripherial, you will need to match these
    SpiChnOpen(spiChn, SPI_OPEN_ON | SPI_OPEN_MODE16 | SPI_OPEN_MSTEN | SPI_OPEN_CKE_REV , spiClkDiv);
    
    // the LED
    mPORTASetPinsDigitalOut(BIT_0);
    mPORTASetBits(BIT_0);
    
    // === config the uart, DMA, vref, timer5 ISR =============
    PT_setup();

    // === setup system wide interrupts  ====================
    INTEnableSystemMultiVectoredInt();
  
  // === now the threads ====================

  // init the threads
  PT_INIT(&pt_param);
  PT_INIT(&pt_key);


  // build the sine lookup table
   // scaled to produce values between 0 and 4096
   int i;
   for (i = 0; i < sine_table_size; i++){
         sin_table[i] = (int)(2047*sin((float)i*6.283/(float)sine_table_size));
    }
   
   for (i=0 ; i<10; i++){
       // this computes and stores the frequency table look up for future use
            freq2[i] = (int)((freq2[i]* (float)two32)/Fs ); 
            freq1[i] = (int)((freq1[i]* (float)two32)/Fs );
   
   }
  
   // init the display
  // NOTE that this init assumes SPI channel 1 connections
  tft_init_hw();
  tft_begin();
  // erase
  tft_fillScreen(ILI9340_BLACK);
  //240x320 vertical display
  tft_setRotation(1); // Use tft_setRotation(1) for 320x240
  
  // schedule the threads
  while(1) {
    //PT_SCHEDULE(protothread_param(&pt_param));
    PT_SCHEDULE(protothread_debounce(&pt_key));
  }
} // main