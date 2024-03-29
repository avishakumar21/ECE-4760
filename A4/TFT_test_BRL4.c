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
volatile unsigned int DAC_dataA ;// output value
volatile unsigned int DAC_dataB ;// output value
static  int adc1; //actual beam angle


volatile SpiChannel spiChn = SPI_CHANNEL2 ;	// the SPI channel to use
volatile int spiClkDiv = 2 ; // 20 MHz max speed for this DAC

volatile int P_gain = 1400, D_gain = 17000, I_gain = 4 ;

//PID control stuff
volatile int errorArray [5] = {0, 0, 0, 0, 0}; 
volatile int pidCounter = 0;
volatile int error = 0; 
volatile int pterm;
volatile int iterm;
volatile int dterm;
static int errorSum = 0;
volatile int pid; 

//to obtain desired angle
//read the potentiometer value between 0 and 1023 at 0 30 and -30
static int desired_angle_30 = 598;
static int desired_angle_0 = 515;
static int desired_angle_neg30 = 418;
static int desired_hanging = 243; 
volatile int desired_angle= 508; 

static int last_iterm =0;

volatile int pwm_on_time = 40000; 

volatile int motor_disp = 0; 

static int time =0;
      
// system 1 second interval tick
int sys_time_seconds ;

void __ISR(_TIMER_2_VECTOR, ipl2) Timer2Handler(void)
{
    mT2ClearIntFlag();
    
    
    // TO READ the adc channel
    adc1 = ReadADC10(0); //actual beam angle 
    
    //setting up the demonstration so that when 5 seconds have passed, the beam 
    //angle is 30 degrees
    if (time == 6) {
        errorSum = 0;
        desired_angle= desired_angle_30;
    
    }
    //when 10 seconds have passed, the beam angle is set to negative 30 degrees
    else if (time == 12){
        errorSum = 0;
        desired_angle = desired_angle_neg30;
    
    }
    //when 15 seconds have passed, the beam angle is set back to 0 degrees 
    else if (time == 18) {
        errorSum = 0;
        desired_angle = desired_angle_0;
    
    }
   
 
    //calculate the error of the beam angle by subtracted the actual value from
    //the desired angle 
    error = desired_angle - adc1; 
    //calculate the p term of PID by multiplying the proportional gain with the 
    //error
     pterm = P_gain * error;

    //calculate the d term of PID by multiplying the derivative gain with the 
     //difference between the current error and the previous error (from 5 times 
     //ago)
    dterm = D_gain * (error - errorArray[pidCounter]);  
    
    //update the errorArray with the current error
    errorArray[pidCounter]= error;
    
    //increment the pidCounter (which indexes in the errorArray) and mod it by 5 
    //so the index ranges from 0-4 only
    pidCounter = (pidCounter+ 1)%5; 

    
    //update errorSum so that it is 0 if error is ever equal to 0
    if(error == 0){
        errorSum = 0;
        //calculate the i term of PID by multiplying the previous iterm by a small 
        //number
        iterm = 0.95*last_iterm; // multiply by 0.9 so that iterm does not entirely drop to 0

    }
    else{
        //update errorSum
        errorSum = errorSum + (error>>I_gain);
        //update iterm and the previous iterm
        iterm = errorSum; 
        last_iterm = iterm;
    }
     
       
     
    //calculate the PID term 
    int pid = pterm + dterm+ iterm; 
    

    //ensures that PID value is between 0-40000 because that is what the motor
    //can effectively read values
    if (pid < 0){
        pid = 0;
    }
    if (pid > 40000){
        pid = 40000; 
    }
   
    //write to potentiometer that drives the angle
    pwm_on_time = pid;  
    
    //drive the motor 
    SetDCOC3PWM(pwm_on_time);
    
    
    //digital low pass filter for motor
    motor_disp = motor_disp + ((pwm_on_time - motor_disp)>>4); 
    

    //write to the DAC
    DAC_dataA = adc1 << 2  ; //(actual beam angle) 
    DAC_dataB = motor_disp>>4; // (motor control signal) 
     
    // CS low to start transaction
     mPORTBClearBits(BIT_4); // start transaction
     // write to spi2
     WriteSPI2(DAC_config_chan_A | DAC_dataA);
    // test for done
    while (SPI2STATbits.SPIBUSY); // wait for end of transaction
     // CS high
     mPORTBSetBits(BIT_4); // end transaction
     
     // CS low to start transaction
     mPORTBClearBits(BIT_4); // start transaction
     // write to spi2
     WriteSPI2(DAC_config_chan_B | DAC_dataB); //write to channel B
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
int counter=0 ;
//initialize values 
static int state=0, poten=0, button1=0, button2=0;

char buffer [60];

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
    while(1) {

        if (counter == 0){
           // poten = ReadADC10(1);
            //updates the gains value on the screen every 1 second
            tft_fillRoundRect(0,10, 100, 14, 1, ILI9340_BLACK);// x,y,w,h,radius,color
            tft_setCursor(0, 10);
            tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
            sprintf(buffer,"P : %d", P_gain);
            tft_writeString(buffer);
           
            
            tft_fillRoundRect(0,30, 100, 14, 1, ILI9340_BLACK);// x,y,w,h,radius,color
            tft_setCursor(0, 30);
            tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
            sprintf(buffer,"D : %d", D_gain);
            tft_writeString(buffer);
           
            
            tft_fillRoundRect(0,50, 100, 14, 1, ILI9340_BLACK);// x,y,w,h,radius,color
            tft_setCursor(0, 50);
            tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
            sprintf(buffer,"I : %d", I_gain);
            tft_writeString(buffer);
            
            //updates the error value on the screen every 1 second

            tft_fillRoundRect(0,100, 100, 14, 1, ILI9340_BLACK);// x,y,w,h,radius,color
            tft_setCursor(0, 100);
            tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
            sprintf(buffer,"er: %d", error);
            tft_writeString(buffer);
            
            //updates the desired angle value on the screen every 1 second

            tft_fillRoundRect(0,80, 150, 14, 1, ILI9340_BLACK);// x,y,w,h,radius,color
            tft_setCursor(0, 80);
            tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
            sprintf(buffer,"Theta: %d", desired_angle);
            tft_writeString(buffer);
            
            //increments time
            time++;
        }
        

        
    //read button press 
    button1 = mPORTAReadBits(BIT_3); 
    button2 = mPORTAReadBits(BIT_4);
    
 //if the button1 is pressed, update the state   
 if (button1 != 0 ){
        state = (state + 1) % 4;
    
    }
 //if button2 is pressed, update the gains term 
 else if (button2 != 0){
         poten = ReadADC10(1); // reading in the user input
         if (state == 0){
         //we're setting the P - gains term
             P_gain = 2*poten; // sets P the range from 0 to 2000
         }
         else if (state == 1){
         //we're setting I - gains term 
             I_gain = poten >>8; //sets I offset from 0 to 4
             
         }
         else if (state == 2) {
         //we're setting D - gains term e^10adc units per factor of e
             D_gain = poten*20; //sets P to the range 0 to 20,460 by increments of 20
         }
         else {

             if (poten < 418 ){ desired_angle = 418;}
             
             else if (poten > 598) {desired_angle = 598;}
             else { desired_angle = poten;}
             
             
         }
    
    }
    //increment timer 
    counter = (counter +1) % 30;
    PT_YIELD_TIME_msec(32);
    // NEVER exit while
  } // END WHILE(1)
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
    SetChanADC10( ADC_CH0_NEG_SAMPLEA_NVREF | ADC_CH0_POS_SAMPLEA_AN1 | ADC_CH0_NEG_SAMPLEB_NVREF | ADC_CH0_POS_SAMPLEB_AN5 );

    OpenADC10( PARAM1, PARAM2, PARAM3, PARAM4, PARAM5 ); // configure ADC using the parameters defined above

    EnableADC10(); // Enable the ADC

    // TO READ the channel
    // adc1 = ReadADC10(0);
    ///////////////////////////////////////////////////////

}
// === Main  ======================================================
void main(void) {
 //SYSTEMConfigPerformance(PBCLK);
  
  ANSELA = 0; ANSELB = 0; 

 // === Config timer and output compare to make PWM ======== 
    // set up timer2 to generate the wave period every  1 mSec!
    OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_1, 40000); //generate period
    // Need ISR to compute PID controller 
    ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_2); 
    mT2ClearIntFlag(); // and clear the interrupt flag 
    
    // set up compare3 for PWM mode 
    OpenOC3(OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE , pwm_on_time, pwm_on_time); 
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
    
    PPSOutput(2, RPB5, SDO2);
    PPSOutput(4, RPB10, SS2);
    SpiChnOpen(SPI_CHANNEL2, SPI_OPEN_ON | SPI_OPEN_MODE16 | SPI_OPEN_MSTEN | SPI_OPEN_CKE_REV , 2);
    
  // end DAC setup
    
  // === config threads ==========
  // turns OFF UART support and debugger pin, unless defines are set
    PT_setup();
  
    //Two ADC channels (one for angle sensor, one for parameter setting)
    set_up_ADC();
    
    //RA4 and RA3 push button set up 
    mPORTASetPinsDigitalIn(BIT_3|BIT_4);
    EnablePullDownA( BIT_3 | BIT_4);

    

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
