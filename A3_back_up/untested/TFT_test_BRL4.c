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
#include <stdlib.h>
////////////////////////////////////
// defines the DAC control bits
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

#define  ball_num  50 // number of balls
// Ball Struct 
typedef struct ball {
    _Accum velocity_x;
    _Accum velocity_y;
    _Accum position_x;
    _Accum position_y;
    int hitCount;
} ball;

static ball balls[ball_num]; // creates an array of  ball structs

_Accum ball_radius = (_Accum)(2);//stores the radius of the balls

// update a 1 second tick counter
static _Accum xc=(_Accum)(325), yc=(_Accum)(120);
static _Accum drag = (_Accum)(.0001);
volatile int current_poten, past_poten=0;// used to save the current and the past_potentiometer values
static float new_poten = 0;
static int points = 0;
#define abs_accum(x) ((x>0)?x:(-x))
static PT_THREAD (protothread_anim(struct pt *pt))
{
    PT_BEGIN(pt);
//       tft_fillRoundRect(0,10, 100, 14, 1, ILI9340_BLACK);// x,y,w,h,radius,color
//        tft_setCursor(0, 10);
//        tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
//        sprintf(buffer,"%f", ((float)rand()/RAND_MAX));
//        tft_writeString(buffer);
    
    //Sets up our balls array 
    static int j=0;
    while(j<ball_num){
        // populates the balls with vel y, vel x, pos x, pos y , hit_counter
       	balls[j].velocity_x = (_Accum)(-1+(float)rand()*-2/RAND_MAX); 
    	balls[j].velocity_y = (_Accum)(-1.5 +(float)rand()*3/RAND_MAX);
    	balls[j].position_x = xc;
    	balls[j].position_y = yc;
        balls[j].hitCount = 0;
        j = j+1;
    }
    
      while(1) {

        // yield time 32 mSec
        PT_YIELD_TIME_msec(32);
        
        // reads and updates the paddle position
        new_poten = ReadADC10(0);
        current_poten = (((new_poten * 214))/1023  + 3); // reads the new potentiometer location
        //updating the paddle drawing
        if (current_poten != past_poten){  
            //avoids un-needed redraws of the paddle 
            tft_reDrawPaddle (past_poten,current_poten,20);
            past_poten = current_poten;
        }
        
      // Balls Render Loop
       static int l;
       static int j;
       for (l = 0; l < ball_num; l++){

            // erase ball
            tft_drawSmallCircle((int)balls[l].position_x, (int) balls[l].position_y, ILI9340_BLACK); //x, y, radius, color
            
            // updates drag compo
            balls[l].velocity_x = balls[l].velocity_x - (balls[l].velocity_x* drag); 
    		balls[l].velocity_y = balls[l].velocity_y - (balls[l].velocity_y* drag); 
            // updates the position of the ball
            balls[l].position_x = balls[l].position_x + balls[l].velocity_x;
            balls[l].position_y = balls[l].position_y + balls[l].velocity_y;
            
            
         if (balls[l].position_y <(_Accum)(4)){
             balls[l].velocity_y  = -balls[l].velocity_y ;  
             balls[l].position_y  = (_Accum)(8) -balls[l].position_y ;
         
       
         }
         else if ( balls[l].position_y >(_Accum)(236)) {   
             balls[l].velocity_y  = -balls[l].velocity_y ;
             balls[l].position_y  = (_Accum)(472)-  balls[l].position_y;}
         
         if ( balls[l].position_x <(_Accum)(4)){ 
//             balls[l].velocity_x = -balls[l].velocity_x;
//             balls[l].position_x = (_Accum)(8) -  balls[l].position_x;
             //failed to hit the ball with the paddle now reset
             balls[l].velocity_x = (_Accum)(-1+(float)rand()*-2/RAND_MAX); 
             balls[l].velocity_y = (_Accum)(-1.5 +(float)rand()*3/RAND_MAX);
             balls[l].position_x = xc;
             balls[l].position_y = yc;
             balls[l].hitCount = 0;
         
         }
         else if ( balls[l].position_x>(_Accum)(316)){  
             balls[l].position_x = (_Accum)(632)- balls[l].position_x;
             balls[l].velocity_x = -balls[l].velocity_x; }
         else if ( (balls[l].position_x> (_Accum)(76) && balls[l].position_x< (_Accum)(86))&&
                 (balls[l].position_y < (_Accum)(60) || balls[l].position_y> (_Accum)(180))){
            //ditch deflection it needs conditional for recycling
             if (( balls[l].position_x -balls[l].velocity_x )< (_Accum)(82)){
                balls[l].velocity_x = (_Accum)(-1+(float)rand()*-2/RAND_MAX); 
                balls[l].velocity_y = (_Accum)(-1.5 +(float)rand()*3/RAND_MAX);
                balls[l].position_x = xc;
                balls[l].position_y = yc;
                balls[l].hitCount = 0;
                points += 1;
                tft_drawSmallCircle((int)balls[l].position_x, (int) balls[l].position_y, ILI9340_RED); //x, y, radius, color
                continue;
             
             }
             else {
                balls[l].velocity_x = -balls[l].velocity_x; 
                balls[l].position_x = (_Accum)(172)- balls[l].position_x;    
             }
         }
         else if ( balls[l].position_x< (_Accum)(45) &&  balls[l].position_x> (_Accum)(37) && 
                 balls[l].position_y > (_Accum)( (past_poten - 5)) && balls[l].position_y < (_Accum)((past_poten +25))){
             
            	balls[l].velocity_x = (_Accum)(-1+(float)rand()*-2/RAND_MAX); 
                balls[l].velocity_y = (_Accum)(-1.5 +(float)rand()*3/RAND_MAX);
                balls[l].position_x = xc;
                balls[l].position_y = yc;
                balls[l].hitCount = 0;
                points += 1;
                tft_drawSmallCircle((int)balls[l].position_x, (int) balls[l].position_y, ILI9340_RED); //x, y, radius, color
                continue;
                
         }
            //checking for ball to ball collision
            for (j=l+1; j<ball_num; j++){
               _Accum dx = abs_accum(balls[l].position_x - balls[j].position_x);
               _Accum dy = abs_accum(balls[l].position_y - balls[j].position_y);
               if (dx<4 && dy<4){
                   _Accum rij = (dx*dx+dy*dy);
                   if ( rij < ((_Accum)4) && balls[l].hitCount == 0){
                       _Accum dvx = balls[l].velocity_x - balls[j].velocity_x; // difference in x velocity
                       _Accum dvy = balls[l].velocity_y - balls[j].velocity_y; // difference in y velocity
                       
                       //CHECK RIJ < 4
                       if (rij>4){
                            _Accum scalar_mult = (dx* dvx +  dy* dvy)/ (rij);//scalar multiple

                            _Accum del_vx = -dx*scalar_mult; 
                            _Accum del_vy = -dy*scalar_mult; 

                            //adds to the velocity of b1
                            balls[l].velocity_x += del_vx;
                            balls[l].velocity_y += del_vy;

                            //subtracts from velocity of b2
                            balls[j].velocity_x -= del_vx;
                            balls[j].velocity_y -= del_vy;

                      
                       }
                       else {
                           //case where balls are too close to each other
                           _Accum temp_vx = balls[l].velocity_x;
                           _Accum temp_vy = balls[l].velocity_y;
                           //reflects their velocities 
                           balls[l].velocity_x = balls[j].velocity_x;
                           balls[l].velocity_y = balls[j].velocity_y;
                           
                            balls[j].velocity_x = temp_vx;
                            balls[j].velocity_y = temp_vy;
                            
                       }
                      // guessing what the value to avoid capture is
                      balls[l].hitCount = 3;
                      
                   }
                   else if (balls[l].hitCount > 0){
                       balls[l].hitCount -= 1;
                    }
               
               
               }
            
            
            }
         //  draw ball
            tft_drawSmallCircle((int)balls[l].position_x, (int) balls[l].position_y, ILI9340_RED); //x, y, radius, color
        }
      } // END WHILE(1)
  PT_END(pt);
} // animation thread

// === Main  ======================================================
void main(void) {

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

  PT_setup();

  // === setup system wide interrupts  ========
  INTEnableSystemMultiVectoredInt();
  
  
  //Sine table 
   int sine_table_size = 256; int i;
    for (i = 0; i < sine_table_size; i++){
         sin_table[i] =  ( DAC_config_chan_A|((int)(1028*sin((float)i*6.283/(float)sine_table_size))));
    }
  
        int timer_limit= 909;
        PPSOutput(2, RPB5, SDO2);
        PPSOutput(4, RPB10, SS2);
        SpiChnOpen(SPI_CHANNEL2,SPI_OPEN_ON | SPI_OPEN_MODE16 | SPI_OPEN_MSTEN | SPI_OPEN_CKE_REV | SPICON_FRMEN | SPICON_FRMPOL, 2);
        OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_1, timer_limit);
        mT2ClearIntFlag();
        #define dmaChn 0
         
        DmaChnOpen(dmaChn, 0, DMA_OPEN_AUTO);

        // set the transfer parameters: source & destination address, source & destination size, number of bytes per event
        // Setting the last parameter to one makes the DMA output one byte/interrupt
          DmaChnSetTxfer(dmaChn, sin_table, (void*)&SPI2BUF, 2*sine_table_size, 2, 2);

         // set the transfer event control: what event is to start the DMA transfer
         // In this case, timer2
         DmaChnSetEventControl(dmaChn, DMA_EV_START_IRQ(_TIMER_2_IRQ));

         DmaChnEnable(dmaChn);


   
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
  
  srand(new_poten);//seeds the random number
  
  past_poten = (((new_poten * 214))/1023  + 3); // reads the new potentiometer location
  
  //drawing the paddle
  tft_fillRect(40, (past_poten), 2, 20 , ILI9340_WHITE);
  
        
  
   
  // round-robin scheduler for threads
  while (1){
      PT_SCHEDULE(protothread_anim(&pt_anim));
      }
  } // main

// === end  ======================================================
