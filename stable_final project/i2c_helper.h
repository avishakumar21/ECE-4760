
#define USE_AND_OR	// To enable AND_OR mask setting for I2C. 
//#include <i2c.h>
#include "plib.h"
#include "tft_master.h"

#define SLAVE_ADDRESS 0x38
#define ACCEL_XOUT_H 0x01
#define ACCEL_XOUT_L 0x02
#define ACCEL_YOUT_H 0x03
#define ACCEL_YOUT_L 0x04
#define ACCEL_ZOUT_H 0x05
#define ACCEL_ZOUT_L 0x06



// Wait by executing nops
void i2c_wait(unsigned int cnt)
{
	while(--cnt)
	{
		asm( "nop" );
		asm( "nop" );
	}
}

// Write a number of chars from data specified by num to the specified address
void i2c_write(char address, char *data, int num)
{
    char i2c_header[2];
    i2c_header[0] = SLAVE_ADDRESS | 0;	//device address & WR
	i2c_header[1] = address;            //register address

    StartI2C1();	//Send the Start Bit
	IdleI2C1();		//Wait to complete

    int i;
	for(i = 0; i < num + 2; i++)
	{
        if(i < 2)
            MasterWriteI2C1( i2c_header[i] );
        else
            MasterWriteI2C1( data[i - 2] );
		IdleI2C1();		//Wait to complete

		//ACKSTAT is 0 when slave acknowledge, 
		//if 1 then slave has not acknowledge the data.
		if( I2C1STATbits.ACKSTAT )
			break;
	}
    
    StopI2C1();	//Send the Stop condition
	IdleI2C1();	//Wait to complete
}

// Read a char from the register specified by address
char i2c_read(char address)
{
    char i2c_header[2];
    i2c_header[0] = ( SLAVE_ADDRESS | 0 );	//device address & WR
	i2c_header[1] = address;                //register address

    StartI2C1();	//Send the Start Bit
	IdleI2C1();		//Wait to complete

    int i;
	for(i = 0; i < 2; i++)
	{
        MasterWriteI2C1( i2c_header[i] );
		IdleI2C1();		//Wait to complete

		//ACKSTAT is 0 when slave acknowledge, 
		//if 1 then slave has not acknowledge the data.
		if( I2C1STATbits.ACKSTAT )
        {
			break;
        }
	}
    
    //now send a start sequence again
	RestartI2C1();	//Send the Restart condition
	i2c_wait(10);
	//wait for this bit to go back to zero
	IdleI2C1();	//Wait to complete

	MasterWriteI2C1( SLAVE_ADDRESS | 1 ); //transmit read command
	IdleI2C1();		//Wait to complete

// read some bytes back
//	MastergetsI2C1(num, dataBuf, 20);
    char data = MasterReadI2C1();
	
	IdleI2C1();	//Wait to complete
    
    StopI2C1();	//Send the Stop condition
	IdleI2C1();	//Wait to complete
    
    return data;
}

// Read three-axis accelerometer and three-axis gyroscope from MPU 6050
// Return values in array pointed to by values
void readImuValues(float* values)
{
//    int statusF = (int) i2c_read(0x00);
    int xAccelH = (int) i2c_read(ACCEL_XOUT_H);
    int xAccelL = (int) i2c_read(ACCEL_XOUT_L);
    int yAccelH = (int) i2c_read(ACCEL_YOUT_H);
    int yAccelL = (int) i2c_read(ACCEL_YOUT_L);
    int zAccelH = (int) i2c_read(ACCEL_ZOUT_H);
    int zAccelL = (int) i2c_read(ACCEL_ZOUT_L);


    values[0] = (float)((xAccelH  << 8) + xAccelL);
    values[1] = (float)((yAccelH << 8) + yAccelL);
    values[2] = (float)((zAccelH << 8) + zAccelL);
//    values[3] = (float)(statusF);

}
void i2_c_setup(void){
    //Enable channel
    //BRG computed value for the baud rate generator. The value is
    // calculated as follows: BRG = (Fpb / 2 / baudrate) - 2.
   OpenI2C1( I2C_ON, 0x0C2 );
   
     
  char data1[] = {0x00};
  i2c_write(0x0E, data1, 1);// sets the accelerometer mode 2G with high pass filter enabled

    
    // 0x10
  char data[] = {0x00};
  i2c_write(0x2C, data, 1); // sends the wake up signal to put in stand by
  
  
  char data0[] = {0x01};
  i2c_write(0x11, data0, 1); // sets  orientation detection to active 
  
  char data2[] = {0x01};
  i2c_write(0x2A,data2,1);//takes in from standby and puts in active mode
  


}

