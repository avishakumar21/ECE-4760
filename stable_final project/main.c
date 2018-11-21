////////////////////////////////////
// clock AND protoThreads configure!
#include "config.h"
// threading library
#include "pt_cornell_1_2_1.h"

////////////////////////////////////
// graphics libraries
#include "tft_master.h"
#include "tft_gfx.h"
// need for rand function
#include <stdlib.h>
#include "math.h"

// need for accelerometer reading
#include "i2c_helper.h"
////////////////////////////////////

static struct pt pt_send, pt_motor;
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

//== Timer 2 interrupt handler ===========================================
// system 1 second interval tick

volatile int pwm_on_time = 20000;



void __ISR(_TIMER_2_VECTOR, ipl2) Timer2Handler(void)
{
    mT2ClearIntFlag();
    
    //drive the motor 
    SetDCOC3PWM(pwm_on_time);
    
   
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



// === Accelerometer Thread =================================================
//used for reading in the accelerometer values
float values[3];// used for reading the accelerometer values
float accum_values[3];//filter values that can be transmitted via bluetooth
static PT_THREAD (protothread_acel(struct pt *pt))
{
    PT_BEGIN(pt);
   

 //  int active = i2c_read( 0x01); // reads if PT is Active

   
    while(1) {
        // yield time 1 second
        PT_YIELD_TIME_msec(200);// reads the value every 200 ms
      
        readImuValues(values); // reads the 3D accelerometer values
        
        
        // creating a digital low pass filter
        accum_values[0] = accum_values[0]*0.99 + .01* values[0]/MAX_G_VALUE;
        accum_values[1] = accum_values[1]*0.99 + .01* values[1]/MAX_G_VALUE;
        accum_values[2] = accum_values[2]*0.99 + .01* values[2]/MAX_G_VALUE;
        
         float roll = atan2(accum_values[1] , accum_values[2]); // computes the roll angle 
         float pitch = atan2((- accum_values[0]) , sqrt(accum_values[1] * accum_values[1] + accum_values[2] * accum_values[2]));//computes the pitch
         
         
         // Used for Debuging Accelerometer
       tft_fillRoundRect(0,10, 300, 14, 1, ILI9340_BLACK);// x,y,w,h,radius,color
       tft_setCursor(0, 10);
       tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(1);
       sprintf(buffer,"roll: %.1f  pitch:%.1f z:%.1f",(roll),
                       (pitch),(values[2]/MAX_G_VALUE));
       tft_writeString(buffer);
        
       
      } // END WHILE(1)
  PT_END(pt);
} // accelerometer thread



void main(void) {
 //SYSTEMConfigPerformance(PBCLK);
  
  ANSELA = 0; ANSELB = 0; 

  // === config threads ==========
  // turns OFF UART support and debugger pin, unless defines are set
  PT_setup();
  
  // === setup accelerometer reading ==========
  i2_c_setup();

  // === setup system wide interrupts  ========
  INTEnableSystemMultiVectoredInt();
  
  
  PT_INIT(&pt_send);
  PT_INIT(&pt_motor);
  
  
   while (1){
      //enables the accelerometer reading
      PT_SCHEDULE(protothread_acel(&pt_accel));

      }
}