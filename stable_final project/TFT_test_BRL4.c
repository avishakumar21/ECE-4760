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
#include <stdio.h>
#include <string.h>
#include "math.h"

// need for accelerometer reading
#include "i2c_helper.h"

// Default Accelerometer max 1g value
#define MAX_G_VALUE 5000

//PORT B
#define EnablePullDownB(bits) CNPUBCLR=bits; CNPDBSET=bits;
#define DisablePullDownB(bits) CNPDBCLR=bits;
#define EnablePullUpB(bits) CNPDBCLR=bits; CNPUBSET=bits;
#define DisablePullUpB(bits) CNPUBCLR=bits;

//PORT A
#define EnablePullDownA(bits) CNPUACLR=bits; CNPDASET=bits;
#define DisablePullDownA(bits) CNPDACLR=bits;
#define EnablePullUpA(bits) CNPDACLR=bits; CNPUASET=bits;
#define DisablePullUpA(bits) CNPUACLR=bits;
// string buffer
char buffer[60];//used for debugging

// === thread structures ============================================
// thread control structs
// note that UART input and output are threads
static struct pt pt_timer, pt_color, pt_anim,pt_DMA_output ;

volatile float pitch,roll;


//== Timer 2 interrupt handler ===========================================
// system 1 second interval tick

volatile int pwm_on_time = 20000;

// used for setting the H bridge to stop the motor from moving
void stop(void){
    
   mPORTAClearBits(BIT_3);//clears AIN0
   mPORTAClearBits(BIT_4);//clears AIN1


}

//used for setting the H bridge direction to forward
void moveforward(void){
    //set AIN0 to ground and AIN1 to VCC
   mPORTAClearBits(BIT_3);//clears AIN0
   mPORTAClearBits(BIT_4);//clears AIN1
   
   mPORTAToggleBits(BIT_4); //sets AI1 to VCC
 }

//used for setting the H bridge direction to backwards
void movebackward(void){
    //set AIN0 to VCC and AIN1 to ground
    mPORTAClearBits(BIT_3);//clears AIN0
    mPORTAClearBits(BIT_4);//clears AIN1
    
    mPORTAToggleBits(BIT_3); //sets AI0 to VCC
}

void extract_pitch_and_roll(void){

    char delim[] = " ";
        
    char *ptr = strtok(PT_term_buffer, delim);//splits up the given input by spaces
    
    if (ptr != NULL){
        sscanf(ptr, "%f", &pitch);//extracts pitch 

        ptr = strtok(NULL,delim);//swaps it to extract roll

        if (ptr !=NULL ){  
            sscanf(ptr, "%f", &roll);//extracts roll
        }
    }
}



// system 1 second interval tick
int sys_time_seconds ;
volatile float extraction1, extraction2; // used to get the float values
char ex1,ex2;//used to read the extraction value

// === Receiver Thread =================================================
// updates the pitch and roll command
static PT_THREAD (protothread_receiver(struct pt *pt))
{
    PT_BEGIN(pt);
    
    sprintf(PT_send_buffer, "%s","AT+RENEW");//resets the bluetooth module
    PT_SPAWN(pt, &pt_DMA_output, PT_DMA_PutSerialBuffer(&pt_DMA_output));
    PT_YIELD_TIME_msec(200) ;
    
    sprintf(PT_send_buffer, "%s","AT+BAUD4");//sets the baud rate to 9600 
    PT_SPAWN(pt, &pt_DMA_output, PT_DMA_PutSerialBuffer(&pt_DMA_output));
    PT_YIELD_TIME_msec(200) ;
    
    PT_terminate_char = '\n'; //sets up the termination char
    PT_terminate_count= 0;
    PT_terminate_time=0;
    
    
     PT_YIELD_TIME_msec(3000) ; //delays read for 3 seconds to give bluetooth time to establish pairing
    
    
    while(1) {
        PT_YIELD_TIME_msec(50);// reads the value every 50 ms
      
       
        tft_fillRoundRect(50,50, 200, 14, 1, ILI9340_BLACK);// x,y,w,h,radius,color
        tft_setCursor(50, 50);
        tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
        sprintf(buffer,"Time:%d",sys_time_seconds);
        tft_writeString(buffer);
        
     
       // PT_SPAWN(pt, &pt_DMA_output, PT_GetMachineBuffer(&pt_DMA_output)); // reads in the latest pitch and roll
        
       // extract_pitch_and_roll(); // extracts the pitch and roll
        
        //test0
        scanf("%c %f %c %f", &ex1,&extraction1, &ex2, &extraction2);// reading in the uart2 channel
        
        //extracts out and assigns the values
        if (ex1 == 'r'){
            roll = extraction1;
        }
        else if (ex1=='p'){
            pitch = extraction1;
        } 

        if (ex2 == 'r'){
            roll = extraction2;
        }
        else if (ex2=='p'){
            pitch = extraction2;
        } 
        
        //test1 set braudrate to 4800
        
        //test2
        //PT_SPAWN(pt, &pt_DMA_output, ble_uart_readln(&pt_DMA_output, '\r'));
        
        //test3 use event triggered interrupt
       
        
        // displays roll and pitch
        tft_fillRoundRect(0,10, 200, 14, 1, ILI9340_BLACK);// x,y,w,h,radius,color
        tft_setCursor(0, 10);
        tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
        sprintf(buffer,"P:%.1f  R:%.1f", pitch, roll);
        tft_writeString(buffer);
        
        sys_time_seconds = (sys_time_seconds+1)%1000;
         PT_YIELD_TIME_msec(30);
        // NEVER exit while
      } // END WHILE(1)
  PT_END(pt);
} // timer thread



// === Accelerometer Thread =================================================
//used for reading in the accelerometer values
float values[3];// used for reading the accelerometer values
float accum_values[3];//low pass filter values that can be transmitted via bluetooth
static PT_THREAD (protothread_acel(struct pt *pt))
{
    PT_BEGIN(pt);
   
 //  int active = i2c_read( 0x01); // reads if PT is Active useful for debugging
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
    
    sprintf(PT_send_buffer, "%s","AT+BAUD4");//sets the baud rate to 9600 
    PT_SPAWN(pt, &pt_DMA_output, PT_DMA_PutSerialBuffer(&pt_DMA_output));
    PT_YIELD_TIME_msec(400) ;
    
    sprintf(PT_send_buffer, "%s","AT+CON3415132D7911");//try connecting to the device ID
    PT_SPAWN(pt, &pt_DMA_output, PT_DMA_PutSerialBuffer(&pt_DMA_output));
    PT_YIELD_TIME_msec(1000) ;

    while(1) {
        // yield time 1 second
        PT_YIELD_TIME_msec(100);// reads the value every 100 ms
      
        readImuValues(values); // reads the 3D accelerometer values
        
        
        // creating a digital low pass filter
        accum_values[0] = accum_values[0]*0.79 + .21* values[0]/MAX_G_VALUE;
        accum_values[1] = accum_values[1]*0.79 + .21* values[1]/MAX_G_VALUE;
        accum_values[2] = accum_values[2]*0.79 + .21* values[2]/MAX_G_VALUE;
        
        //roll and pitch range between 1.5 and -1.5
        float roll = atan2(accum_values[1] , accum_values[2]); // computes the roll angle 
        float pitch = atan2((- accum_values[0]) , sqrt(accum_values[1] * accum_values[1] + accum_values[2] * accum_values[2]));//computes the pitch
         
         
//       // Used for Debugging Accelerometer
//       tft_fillRoundRect(0,10, 300, 14, 1, ILI9340_BLACK);// x,y,w,h,radius,color
//       tft_setCursor(0, 10);
//       tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(1);
//       sprintf(buffer,"roll: %.1f  pitch:%.1f z:%.1f",(roll),
//                       (pitch),(values[2]/MAX_G_VALUE));
//       tft_writeString(buffer);
       
       
       sprintf(PT_send_buffer, "p %.1f r %.1f \r", pitch, roll);
       PT_SPAWN(pt, &pt_DMA_output, PT_DMA_PutSerialBuffer(&pt_DMA_output));
       
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
  mPORTASetPinsDigitalOut(BIT_3);    //Set port as output for AIN0
  mPORTASetPinsDigitalOut(BIT_4);    //Set port as output for AIN1

  // init the threads
  PT_INIT(&pt_timer);
  PT_INIT(&pt_color);
  PT_INIT(&pt_DMA_output);

  // REMEMBER TO SWAP THE RX LINE To RPB1 instead of RPB8
  // ALSO COMMENT OUT TFT INIT CODE WHEN LOADING ON SMALL BOARD
  // init the display
  tft_init_hw();
  tft_begin();
  tft_fillScreen(ILI9340_BLACK);
  //240x320 vertical display
  tft_setRotation(0); // Use tft_setRotation(1) for 320x240

  //set up i2C
  //i2_c_setup();

  

  
  // round-robin scheduler for threads
  while (1){
      // PT_SCHEDULE(protothread_acel(&pt_color));
      PT_SCHEDULE(protothread_receiver(&pt_color));
     
      }
  } // main

// === end  ======================================================
