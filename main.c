#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include <stm32f4xx_i2c.h>
//#include "pitch_stabilize.h>
//#include <stdio.h>

#define SLAVE_ADDRESS 0x3D 
void Delay(__IO uint32_t time);
extern  __IO uint32_t timingDelay;


void I2C_init(void){

	    /*
         *Description
         *define varibles of GPIOx& I2Cx*/
	GPIO_InitTypeDef GPIO_InitStruct;
	I2C_InitTypeDef I2C_InitStruct;
        I2C_InitTypeDef I2C_InitStruct2;
        I2C_InitTypeDef I2C_InitStruct3;
        
        
	    /*
         *Description
         *extra to check i2c working*/
        GPIO_InitTypeDef GPIO_Output;     // For some debugging LEDs
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
 
	    /*
         *Description
         *Configure PD12, PD13, PD14 and PD15 in output pushpull mode */
        GPIO_Output.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15 ;
        GPIO_Output.GPIO_Mode = GPIO_Mode_OUT;
        GPIO_Output.GPIO_OType = GPIO_OType_PP;
        GPIO_Output.GPIO_Speed = GPIO_Speed_100MHz;
        GPIO_Output.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_Init(GPIOD, &GPIO_Output);
        /*end of extra to check i2c working*/
        
        /*
         *Description
         *enable APB1 peripheral clock for I2Cx*/
	// enable APB1 peripheral clock for I2C1
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
        // enable APB1 peripheral clock for I2C2
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
         // enable APB1 peripheral clock for I2C3
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C3, ENABLE);
	// enable clock for SCL and SDA pins
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
	/*
     *Description
     *setup SCL and SDA pins
	 * You can connect the I2C1 functions to two different
	 * pins:
	 * 1. SCL on PB6  
	 * 2. SDA on PB7 
	 */
        /* setup SCL and SDA pins
	 * You can connect the I2C2 functions to two different
	 * pins:
	 * 1. SCL on PB10
	 * 2. SDA on PB11
	 */
         /* setup SCL and SDA pins
	 * You can connect the I2C3 functions to two different
	 * pins:
	 * 1. SCL on PA8
	 * 2. SDA on PC9 
	 */
        GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8; 
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;			// set pins to alternate function
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;		        // set GPIO speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;		  	// set output to open drain --> the line has to be only pulled low, not driven high
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;			// enable pull up resistors
	GPIO_Init(GPIOA, &GPIO_InitStruct);			        // init GPIOA
        GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9; 
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;			// set pins to alternate function
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;		        // set GPIO speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;		  	// set output to open drain --> the line has to be only pulled low, not driven high
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;			// enable pull up resistors
	GPIO_Init(GPIOC, &GPIO_InitStruct);			        // init GPIOC
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;			// set pins to alternate function
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;		        // set GPIO speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;		  	// set output to open drain --> the line has to be only pulled low, not driven high
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;			// enable pull up resistors
	GPIO_Init(GPIOB, &GPIO_InitStruct);			        // init GPIOB
        
        /*
         *Description
         *Connect I2Cx pins to AF */
	// Connect I2C1 pins to AF  
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1);	// SCL for I2C1
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1); // SDA for I2C1
        // Connect I2C2 pins to AF  
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_I2C2);   //scl for I2C2
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_I2C2);  //sDa for I2C2
        // Connect I2C3 pins to AF  
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_I2C2);   //scl for I2C3
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_I2C2);  //sDa for I2C3
       
        /*
         *Description
         *configure I2Cx*/
	// configure I2C1 
	I2C_Init(I2C1, &I2C_InitStruct);		        // init I2C1
        // configure I2C2
        I2C_Init(I2C2, &I2C_InitStruct2);	             // init I2C2
        // configure I2C3
        I2C_Init(I2C3, &I2C_InitStruct3);	             // init I2C3
	
        /*
         *Description
         *enable I2Cx*/
	// enable I2C1
	I2C_Cmd(I2C1, ENABLE);
        // enable I2C2
	I2C_Cmd(I2C2, ENABLE);
         // enable I2C3
	I2C_Cmd(I2C3, ENABLE);
}

/*
 *Description
 *This function issues a start condition and 
 * transmits the ress + R/W bit
 * 
 * Parameters:
 * 		I2Cx --> the I2C peripheral e.g. I2C1
 * 		address --> the 7 bit slave address
 * 		direction --> the transmission direction can be:
 * 						I2C_Direction_Tranmitter for Master transmitter mode
 * 						I2C_Direction_Receiver for Master receiver
 */
void I2C_start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction){
	// wait until I2C1 is not busy any more
	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));
  
	// Send I2C1 START condition 
	I2C_GenerateSTART(I2Cx, ENABLE);
	  
	// wait for I2C1 EV5 --> Slave has acknowledged start condition
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));
		
	// Send slave Address for write 
	I2C_Send7bitAddress(I2Cx, address, direction);
	 /*hena hoa mstne al ack tgelo mn el slave fe eve6 */ 
	/* wait for I2Cx EV6, check if 
	 * either Slave has acknowledged Master transmitter or
	 * Master receiver mode, depending on the transmission
	 * direction
	 */ 
	if(direction == I2C_Direction_Transmitter){
		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	}
	else if(direction == I2C_Direction_Receiver){
		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
	}
}

/*
 *Description
 * This function transmits one byte to the slave device
 * Parameters:
 *		I2Cx --> the I2C peripheral e.g. I2C1 
 *		data --> the data byte to be transmitted
 */
void I2C_write(I2C_TypeDef* I2Cx, uint8_t data)
{
	// wait for I2C1 EV8 --> last byte is still being transmitted (last byte in SR, buffer empty), next byte can already be written
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTING));
	I2C_SendData(I2Cx, data);
}

/*
 *Description
 * This function reads one byte from the slave device 
 * and acknowledges the byte (requests another byte)
 */
uint8_t I2C_read_ack(I2C_TypeDef* I2Cx){
	// enable acknowledge of received data
	I2C_AcknowledgeConfig(I2Cx, ENABLE);
	// wait until one byte has been received
	while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) );
	// read data from I2C data register and return data byte
	uint8_t data = I2C_ReceiveData(I2Cx);
	return data;
}

/*
 *Description
 * This function reads one byte from the slave device
 * and doesn't acknowledge the received data 
 * after that a STOP condition is transmitted
 */
uint8_t I2C_read_nack(I2C_TypeDef* I2Cx){
	// disable acknowledge of received data
	// nack also generates stop condition after last byte received
	// see reference manual for more info
	I2C_AcknowledgeConfig(I2Cx, DISABLE);
	I2C_GenerateSTOP(I2Cx, ENABLE);
	// wait until one byte has been received
	while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) );
	// read data from I2C data register and return data byte
	uint8_t data = I2C_ReceiveData(I2Cx);
	return data;
}

/*
 *Description
 *This function issues a stop condition and therefore
 * releases the bus
 */
void I2C_stop(I2C_TypeDef* I2Cx){
	
	// Send I2C1 STOP Condition after last byte has been transmitted
	I2C_GenerateSTOP(I2Cx, ENABLE);
	// wait for I2C1 EV8_2 --> byte has been transmitted
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
}
int initial_pitch_flag = 0; 
uint8_t pitch_first_1;
uint8_t pitch_first_2;
short initial_pitch(void)
{
  
  initial_pitch_flag = 1;
  I2C_init();
  I2C_start(I2C1, SLAVE_ADDRESS<<1, I2C_Direction_Receiver);
  uint8_t temp;
  for( int i=0;i<18;i++)
  {
   temp = I2C_read_ack(I2C1);
  
  }
      pitch_first_1= I2C_read_ack(I2C1);
      pitch_first_2= I2C_read_ack(I2C1);
          I2C_stop(I2C1);
short concatenated = pitch_first_1 | pitch_first_2;
  return concatenated  ;
  

}
short not_initial_pitch()
{

//short ipitch = pitch_first_1 | pitch_first_2;
uint8_t pitch_1_stabilized  = pitch_first_1;
uint8_t pitch_2_stabilized = pitch_first_2;
uint8_t temp;
  for( int i=0;i<18;i++)
  {
   temp = I2C_read_ack(I2C1);
  
  }
  
uint8_t pitch_1_UNstabilized  = I2C_read_ack(I2C1);
uint8_t pitch_2_UNstabilized  = I2C_read_ack(I2C1);
          I2C_stop(I2C1);

// first part 
if (pitch_1_UNstabilized>pitch_1_stabilized)
pitch_1_stabilized = pitch_1_UNstabilized - (pitch_1_UNstabilized - pitch_1_stabilized);
else if (pitch_1_UNstabilized<pitch_1_stabilized)
{
pitch_1_stabilized= pitch_1_UNstabilized +(pitch_1_stabilized-pitch_1_UNstabilized);
}
else if (pitch_1_UNstabilized==pitch_1_stabilized) ;

// second 

if (pitch_2_UNstabilized>pitch_2_stabilized)
pitch_2_stabilized = pitch_2_UNstabilized - (pitch_2_UNstabilized - pitch_2_stabilized);
else if (pitch_2_UNstabilized<pitch_2_stabilized)
{
pitch_2_stabilized= pitch_2_UNstabilized +(pitch_2_stabilized-pitch_2_UNstabilized);
}
else if (pitch_2_UNstabilized==pitch_2_stabilized) ;






short concatenated =pitch_1_stabilized|pitch_2_stabilized;
return concatenated ; 
}




short stabilize_pitch ()
{
if (initial_pitch_flag==0) return  initial_pitch();
else return not_initial_pitch();



}

//////////////////////////////////////////////////////////
int initial_yaw_flag = 0; 
uint8_t yaw_first_1;
uint8_t yaw_first_2;
short initial_yaw(void)
{
  
  initial_yaw_flag = 1;
  I2C_init();
  I2C_start(I2C1, SLAVE_ADDRESS<<1, I2C_Direction_Receiver);
  uint8_t temp;
  for( int i=0;i<22;i++)
  {
   temp = I2C_read_ack(I2C1);
  
  }
      yaw_first_1= I2C_read_ack(I2C1);
      yaw_first_2= I2C_read_ack(I2C1);
          I2C_stop(I2C1);
short concatenated = yaw_first_1 | yaw_first_2;
  return concatenated  ;
  

}
short not_initial_yaw()
{

//short ipitch = pitch_first_1 | pitch_first_2;
uint8_t yaw_1_stabilized  = yaw_first_1;
uint8_t yaw_2_stabilized = yaw_first_2;
uint8_t temp;
  for( int i=0;i<22;i++)
  {
   temp = I2C_read_ack(I2C1);
  
  }
  
uint8_t yaw_1_UNstabilized  = I2C_read_ack(I2C1);
uint8_t yaw_2_UNstabilized  = I2C_read_ack(I2C1);
          I2C_stop(I2C1);

// first part 
if (yaw_1_UNstabilized>yaw_1_stabilized)
yaw_1_stabilized = yaw_1_UNstabilized - (yaw_1_UNstabilized - yaw_1_stabilized);
else if (yaw_1_UNstabilized<yaw_1_stabilized)
{
yaw_1_stabilized= yaw_1_UNstabilized +(yaw_1_stabilized-yaw_1_UNstabilized);
}
else if (yaw_1_UNstabilized==yaw_1_stabilized) ;

// second 

if (yaw_2_UNstabilized>yaw_2_stabilized)
yaw_2_stabilized = yaw_2_UNstabilized - (yaw_2_UNstabilized - yaw_2_stabilized);
else if (yaw_2_UNstabilized<yaw_2_stabilized)
{
yaw_2_stabilized= yaw_2_UNstabilized +(yaw_2_stabilized-yaw_2_UNstabilized);
}
else if (yaw_2_UNstabilized==yaw_2_stabilized) ;






short concatenated =yaw_1_stabilized|yaw_2_stabilized;
return concatenated ; 
}




short stabilize_yaw ()
{
if (initial_yaw_flag==0) return  initial_yaw();
else return not_initial_yaw();



}
//////////////////////////////////////////////////////////



int main(void){
//short test = stabilize_pitch();	
    // printf("r:");
  
  
  
	I2C_init(); // initialize I2C1 & I2C2  & I2C3 peripheral
        
        
        /*Description
         *receive 8 bits data from imu by using I2C1*/
        //el mafrod aktb el slave addres bta3 el imu ely hst2bl meno sata 
        //#define SLAVE_ADDRESS 0x4A // the slave address (example) hykon 7aga shbh dah
        I2C_start(I2C1, SLAVE_ADDRESS<<1, I2C_Direction_Receiver); // start a transmission in Master receiver mode
        
        /*accelerometer read*/
        uint8_t accelx_first_1= I2C_read_ack(I2C1);
        uint8_t accelx_first_2= I2C_read_ack(I2C1);
        uint8_t accely_second_1= I2C_read_ack(I2C1);
        uint8_t accely_second_2= I2C_read_ack(I2C1);
        uint8_t accel_third_1= I2C_read_ack(I2C1);
        uint8_t accel_third_2= I2C_read_ack(I2C1);
        /*gyroscope read*/
         uint8_t gyrox_first_1= I2C_read_ack(I2C1);
         uint8_t gyrox_first_2= I2C_read_ack(I2C1);
         uint8_t gyroy_second_1= I2C_read_ack(I2C1);
         uint8_t gyroy_second_2= I2C_read_ack(I2C1);
         uint8_t gyro_third_1= I2C_read_ack(I2C1);
         uint8_t gyro_third_2= I2C_read_ack(I2C1);
        /*magnometer read*/
          uint8_t magx_first_1= I2C_read_ack(I2C1);
         uint8_t magx_first_2= I2C_read_ack(I2C1);
         uint8_t magy_second_1= I2C_read_ack(I2C1);
         uint8_t magy_second_2= I2C_read_ack(I2C1);
         uint8_t mag_third_1= I2C_read_ack(I2C1);
         uint8_t mag_third_2= I2C_read_ack(I2C1);
         /*Euler angle*/
         uint8_t pitch_first_1= I2C_read_ack(I2C1);
         uint8_t pitch_first_2= I2C_read_ack(I2C1);
         uint8_t roll_second_1= I2C_read_ack(I2C1);
         uint8_t roll_second_2= I2C_read_ack(I2C1);
         uint8_t yaw_third_1= I2C_read_ack(I2C1);
         uint8_t yaw_third_2= I2C_read_ack(I2C1);
         
          /*quaternion used for more accuracy*/
         uint8_t Qw_first_1= I2C_read_ack(I2C1);
         uint8_t Qw_first_2= I2C_read_ack(I2C1);
         uint8_t Qx_second_1= I2C_read_ack(I2C1);
         uint8_t Qx_second_2= I2C_read_ack(I2C1);
         uint8_t Qy_third_1= I2C_read_ack(I2C1);
         uint8_t Qy_third_2= I2C_read_ack(I2C1);
         uint8_t Q_fourth_1= I2C_read_ack(I2C1);
         uint8_t Q_fourth_2= I2C_read_ack(I2C1);
       /*stop bit*/
       I2C_stop(I2C1);
         
       
       
        /*
         *Description
         *after processing this data in the control algorithm it will be send to motors & logger*/
       
	/*
         *Description
         *send 8 bits data from stm to motors using I2C3*/
        //el mafrod aktb el slave addres bta3 el motors ely hb3tlo data
        #define SLAVE_ADDRESS 0x4A // the slave address (example) hykon 7aga shbh dah
         I2C_start(I2C3, SLAVE_ADDRESS<<1, I2C_Direction_Transmitter); // start a transmission in Master transmitter mode
         
          /*pitch*/
        I2C_write(I2C3, pitch_first_1);
        I2C_write(I2C3, pitch_first_2);
        /*roll*/ 
        I2C_write(I2C3, roll_second_1);
        I2C_write(I2C3, roll_second_2);
        /*yaw*/
        I2C_write(I2C3,yaw_third_1);
        I2C_write(I2C3, yaw_third_2);
	/*stop bit*/
        I2C_stop(I2C2);
         
         
         /*
         *Description
         *send 8 bits data from stm to logger using I2C2*/
        //el mafrod aktb el slave addres bta3 el logger ely hb3tlo data
        //#define SLAVE_ADDRESS 0x4A // the slave address (example) hykon 7aga shbh dah
         I2C_start(I2C2, SLAVE_ADDRESS<<1, I2C_Direction_Transmitter); // start a transmission in Master transmitter mode
        
        /*pitch*/
        I2C_write(I2C2, pitch_first_1);
        I2C_write(I2C2, pitch_first_2);
        /*roll*/ 
        I2C_write(I2C2, roll_second_1);
        I2C_write(I2C2, roll_second_2);
        /*yaw*/
        I2C_write(I2C2,yaw_third_1);
        I2C_write(I2C2, yaw_third_2);
	/*stop bit*/
        I2C_stop(I2C2);
        
	return 0;
}
void Delay(__IO uint32_t time)
{
timingDelay=time;
while(timingDelay!=0);

}
