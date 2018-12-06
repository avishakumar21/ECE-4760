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

//used for extracting float from char array
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

//used for pitch and roll computation
#include "math.h"

// need for accelerometer reading
#include "i2c_helper.h"

// Default Accelerometer max 1g value
#define MAX_G_VALUE 5000


//Note that the RX pin needs to be changed when using bigboard and accelerometer
#define BOOL_ACCELEROMETER_PROGRAMMING 1
#define BOOL_BIG_BOARD 0
#define ESTABLISH_PAIRING 0

//PORT A
#define EnablePullDownA(bits) CNPUACLR=bits; CNPDASET=bits;
#define DisablePullDownA(bits) CNPDACLR=bits;
#define EnablePullUpA(bits) CNPDACLR=bits; CNPUASET=bits;
#define DisablePullUpA(bits) CNPUACLR=bits;

// string buffer
char buffer[60];//used for debugging

static char term_buffer [15]; //used for reading in UART VALUES

// === thread structures ============================================
// thread control structs
static struct pt pt_timer, pt_color, pt_anim,pt_DMA_output ;

volatile float pitch,roll;


//== Timer 2 interrupt handler ===========================================
// system 1 second interval tick

volatile int pwm_on_time = 20000;
volatile int pwm_servo = 50000;

// used for setting the H bridge to stop the motor from moving
//used for setting the H bridge direction to forward
void moveforward(void){
    //set AIN0 to ground and AIN1 to VCC
   mPORTAClearBits(BIT_0);//clears AIN0
   mPORTAClearBits(BIT_3);//clears AIN1
   mPORTAToggleBits(BIT_0); //sets AI1 to VCC
   //pwm_on_time = 20000;
 }

//used for setting the H bridge direction to backwards
void movebackward(void){
    //set AIN0 to VCC and AIN1 to ground
    mPORTAClearBits(BIT_0);//clears AIN0
    mPORTAClearBits(BIT_3);//clears AIN1
    mPORTAToggleBits(BIT_3); //sets AI0 to VCC
    //pwm_on_time = 20000;
}


  // used for setting the H bridge to stop the motor from moving
void stop(void){
    
   mPORTAClearBits(BIT_0);//clears AIN0
   mPORTAClearBits(BIT_3);//clears AIN1
   //pwm_on_time = 0;

}
void __ISR(_TIMER_2_VECTOR, ipl2) Timer2Handler(void)
{

//if roll is 1.5-- hand is tilted to left 
//if roll is -1.5-- hand is tilted to right
    
 
  
  //left-> 4500; 
  //straight -> 5000
  //right->5500;
    
    
    //for servo
    if (roll == 0){
        pwm_servo = 5000;
        SetDCOC4PWM(pwm_servo);

    }
    else if (roll > 0){
        pwm_servo = (int) (700*(roll/1.5)) + 5000;
        SetDCOC4PWM(pwm_servo);

    }
    else {
        pwm_servo = (int) (700*(roll/1.5)) + 5000;
        SetDCOC4PWM(pwm_servo);

    }
    
        mT2ClearIntFlag();

    
   }




void __ISR(_TIMER_3_VECTOR, ipl2) Timer3Handler(void)
{
    
//if pitch is 1.5-- hand is completely forward
//if pitch is -1.5 -- hand is completed backward
       //for motor
    if (pitch == 0){
        stop();   
    }
    else if (pitch > 0){
        pwm_on_time = (int)(20000*(pitch/1.5) + 16000);
        moveforward();
        SetDCOC1PWM(pwm_on_time);

    }
    else{
        pwm_on_time = (int) (-20000*(pitch/1.5) + 16000);
        movebackward();
        SetDCOC1PWM(pwm_on_time);

    }
    

    mT3ClearIntFlag();

}


void extract_pitch_and_roll(void){

    char delim[] = " ";
        
    char *ptr = strtok(term_buffer, delim);//splits up the given input by spaces
    ptr = strtok(NULL,delim);//swaps it from the char 'p' to the first pitch value
    
    if (ptr != NULL){
        sscanf(ptr, "%f", &pitch);//extracts pitch 

        ptr = strtok(NULL,delim);//swaps it to extract roll
        ptr = strtok(NULL,delim);//swaps it from the char 'r'
    

        if (ptr !=NULL ){  
            sscanf(ptr, "%f", &roll);//extracts roll
        }
    }
}

// system 1 second interval tick
int sys_time_seconds ;
volatile float extraction1, extraction2; // used to get the float values
static int index; // used to keep track of the extraction index
static char character;//used to extract UART chars
// === Receiver Thread =================================================
// updates the pitch and roll command
static PT_THREAD (protothread_receiver(struct pt *pt))
{
    PT_BEGIN(pt);
    
    if (ESTABLISH_PAIRING) {
        sprintf(PT_send_buffer, "%s","AT+RENEW");//resets the bluetooth module
        PT_SPAWN(pt, &pt_DMA_output, PT_DMA_PutSerialBuffer(&pt_DMA_output));
        PT_YIELD_TIME_msec(200);
    
     }
    
   PT_YIELD_TIME_msec(3000) ; //delays read for 3 seconds to give bluetooth time to establish pairing
    
    
    while(1) {
        
     if (BOOL_BIG_BOARD){
        tft_fillRoundRect(50,50, 200, 14, 1, ILI9340_BLACK);// x,y,w,h,radius,color
        tft_setCursor(50, 50);
        tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
        sprintf(buffer,"T:%d",sys_time_seconds);
        tft_writeString(buffer);
        index = 0;
       }
     
      
      
     memset(PT_term_buffer, 0, 15); // clears the memory of the input buffer
       
      while (index<14){
         if(UART2GetErrors() & 0x02){
            UART2ClearAllErrors();}
          while(!UARTReceivedDataIsAvailable(UART2)){};
          character = (char) UARTGetDataByte(UART2);
          
          if ( (index == 0|| character == '\r' || character == 'X' )&& character != 'p'){
              break;
          }
          else if (term_buffer[index] == 'X'){
              term_buffer[index]=0;
              break;
          }
          else {
              term_buffer[index]=character;
          }
          index +=1;
        
   }
        term_buffer[index]=0;//sets the last index to zero

        
        // displays roll and pitch
        if (index !=0){

 // USED FOR DEBUGGING
//           if (BOOL_BIG_BOARD){
//        
//            tft_fillRoundRect(0,10, 200, 14, 1, ILI9340_BLACK);// x,y,w,h,radius,color
//            tft_setCursor(0, 10);
//            tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
//            sprintf(buffer,"%s %d",term_buffer,index);
//            tft_writeString(buffer);
//            }
        
         extract_pitch_and_roll();//extracts the pitch and roll values based on the input format
         
           if (BOOL_BIG_BOARD){
            tft_fillRoundRect(0,70, 200, 14, 1, ILI9340_BLACK);// x,y,w,h,radius,color
            tft_setCursor(0, 70);
            tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
            sprintf(buffer,"%.1f %.1f",pitch,roll);
            tft_writeString(buffer);
           }
        
        }
        sys_time_seconds = (sys_time_seconds+1)%1000;
        
     
        // NEVER exit while
      } // END WHILE(1)
  PT_END(pt);
} // timer thread



// === Accelerometer Thread =================================================
//used for reading in the accelerometer values
float values[3];// used for reading the accelerometer values
float accum_values[3];//low pass filtered accelerometer values
static PT_THREAD (protothread_acel(struct pt *pt))
{
    PT_BEGIN(pt);
   
 //  int active = i2c_read( 0x01); // reads if PT is Active useful for debugging
    
    if (ESTABLISH_PAIRING) {
    
        sprintf(PT_send_buffer, "%s","AT");//sets up to make sure it's working
        PT_SPAWN(pt, &pt_DMA_output, PT_DMA_PutSerialBuffer(&pt_DMA_output));
        PT_YIELD_TIME_msec(400) ;

        sprintf(PT_send_buffer, "%s","AT+RENEW");//resets the bluetooth module
        PT_SPAWN(pt, &pt_DMA_output, PT_DMA_PutSerialBuffer(&pt_DMA_output));
        PT_YIELD_TIME_msec(400) ;

        sprintf(PT_send_buffer, "%s","AT+ROLE1");//swaps it to master mode
        PT_SPAWN(pt, &pt_DMA_output, PT_DMA_PutSerialBuffer(&pt_DMA_output));
        PT_YIELD_TIME_msec(400) ;

        sprintf(PT_send_buffer, "%s","AT+IMME1");//Wait for a connection command before connecting 
        PT_SPAWN(pt, &pt_DMA_output, PT_DMA_PutSerialBuffer(&pt_DMA_output));
        PT_YIELD_TIME_msec(400) ;

        sprintf(PT_send_buffer, "%s","AT+BAUD0");//sets the baud rate to 9600 
        PT_SPAWN(pt, &pt_DMA_output, PT_DMA_PutSerialBuffer(&pt_DMA_output));
        PT_YIELD_TIME_msec(400) ;

        sprintf(PT_send_buffer, "%s","AT+CON3CA30896A59D");//try connecting to the device ID
        PT_SPAWN(pt, &pt_DMA_output, PT_DMA_PutSerialBuffer(&pt_DMA_output));
         PT_YIELD_TIME_msec(200) ;
    }
   
        mPORTASetPinsDigitalIn(BIT_3);
        EnablePullDownA( BIT_3 );
     PT_YIELD_TIME_msec(4000);// gives bluetooth module 4 seconds to establish pairing 

    while(1) {
        // yield time 1 second
        PT_YIELD_TIME_msec(50);// reads the value every 50 ms
      
        readImuValues(values); // reads the 3D accelerometer values
        
        
        // creating a digital low pass filter
        accum_values[0] = accum_values[0]*0.75 + .25* values[0]/MAX_G_VALUE;
        accum_values[1] = accum_values[1]*0.75 + .25* values[1]/MAX_G_VALUE;
        accum_values[2] = accum_values[2]*0.75 + .25* values[2]/MAX_G_VALUE;
        
        //roll and pitch range between 1.5 and -1.5
        float roll = atan2(accum_values[1] , accum_values[2]); // computes the roll angle 
        float pitch = atan2((- accum_values[0]) , sqrt(accum_values[1] * accum_values[1] + accum_values[2] * accum_values[2]));//computes the pitch
        
        //used to cap the roll and pitch values to -pi/2 and pi/2
        if (roll > 1.5){
            roll = 1.5;
        }
        else if (roll < -1.5){
            roll = -1.5;
        }
        
        if (pitch > 1.5){
            roll = 1.5;
        }
        else if (pitch < -1.5){
            roll = -1.5;
        }
         
        if (BOOL_BIG_BOARD){
          // Used for Debugging Accelerometer
          tft_fillRoundRect(0,10, 300, 14, 1, ILI9340_BLACK);// x,y,w,h,radius,color
          tft_setCursor(0, 10);
          tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(1);
          sprintf(buffer,"roll: %.1f  pitch:%.1f z:%.1f",(roll),
                          (pitch),(values[2]/MAX_G_VALUE));
          tft_writeString(buffer);
        }
       
        int kill =  mPORTAReadBits(BIT_3);
        if ( kill == 0 ){
       //sends the pitch and roll via UART
            sprintf(PT_send_buffer, "p %.1f r %.1f X", pitch, roll);
            PT_SPAWN(pt, &pt_DMA_output, PT_DMA_PutSerialBuffer(&pt_DMA_output));
        }
        else{
            sprintf(PT_send_buffer, "p %.1f r %.1f X", 0.0, 0.0);
            PT_SPAWN(pt, &pt_DMA_output, PT_DMA_PutSerialBuffer(&pt_DMA_output));
        
        }
       
      } // END WHILE(1)
  PT_END(pt);
} // accelerometer thread


// === Main  ======================================================
void main(void) {
 //SYSTEMConfigPerformance(PBCLK);
  
  ANSELA = 0; ANSELB = 0; 
    
  // === config threads ==========
  // turns OFF UART support and debugger pin, unless defines are set
  PT_setup();

  // === setup system wide interrupts  ========
  INTEnableSystemMultiVectoredInt();
  
  //USED for H bridge direction setting
  mPORTASetPinsDigitalOut(BIT_3);    //Set port as output for AIN0 as RPA3
  mPORTASetPinsDigitalOut(BIT_4);    //Set port as output for AIN1 as RPA4

  // init the threads
  PT_INIT(&pt_timer);
  PT_INIT(&pt_color);
  PT_INIT(&pt_DMA_output);

  // REMEMBER TO SWAP THE RX LINE To RPB1 instead of RPB8
  // ALSO COMMENT OUT TFT INIT CODE WHEN LOADING ON SMALL BOARD

  if (BOOL_BIG_BOARD){
      // init the display
        tft_init_hw();
        tft_begin();
        tft_fillScreen(ILI9340_BLACK);
        //240x320 vertical display
        tft_setRotation(0); // Use tft_setRotation(1) for 320x240
    }
  
  //set up i2C
    if (BOOL_ACCELEROMETER_PROGRAMMING){
        //NOTE THAT YOU CAN'T have I2C and RX TX communication 
        //working on the big board on the same time with out 
        //reassigning the RX pin
        i2_c_setup();
    }
    else{
        //=== Timer Setups =========================================
        // Set up Timer 2 to control Servo Motor Direction 
        OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_16, pwm_servo); // 40000000/(50000*16)=50Hz Frequency
        ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_2); 
        mT2ClearIntFlag(); // and clear the interrupt flag

        // set up timer 3 for motor
        OpenTimer3(T3_ON | T3_SOURCE_INT | T3_PS_1_256, pwm_on_time); //Controls PWM Signals    
        ConfigIntTimer3(T3_INT_ON | T3_INT_PRIOR_2); 
        mT3ClearIntFlag(); // and clear the interrupt flag



        //=== Setup of  motor =========================================

        // set up compare1 for PWM mode (motor)
        OpenOC1(OC_ON | OC_TIMER3_SRC | OC_PWM_FAULT_PIN_DISABLE , pwm_on_time, pwm_on_time); 
        // OC1 is PPS group 1, map to RPB3  
        PPSOutput(1, RPB3, OC1);

        //set up compare4 for PWM (servo)
        OpenOC4(OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE , pwm_servo, pwm_servo); 
        //OC4 is PPS group 3, map to RPA4
        PPSOutput(3 , RPA4, OC4);
          // === setup system wide interrupts  ========
          INTEnableSystemMultiVectoredInt();

          //USED for H bridge direction setting
          //0 and 1 for h bridge
        mPORTASetPinsDigitalOut(BIT_0 | BIT_3);    //Set port as output for AIN0
        mPORTASetBits(BIT_0 | BIT_3);

    
    
    }
  
   
  while (1){
     if (BOOL_ACCELEROMETER_PROGRAMMING){
         PT_SCHEDULE(protothread_acel(&pt_color));
      }
      else{
        PT_SCHEDULE(protothread_receiver(&pt_color));
      }
     
    } //end of while
  } // main


