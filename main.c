
#include <stdio.h>
#include <stdint.h>
#include <stm32f4xx.h>
#include <stm32f4xx_i2c.h>
#include <stm32f4xx_spi.h>
#include <stm32f4xx_tim.h>
#include <stm32f4xx_usart.h>
#include <math.h>
#include <stdio.h>
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "defines.h"
#include "tm_stm32f4_delay.h"
#include "tm_stm32f4_disco.h"
#include "tm_stm32f4_keypad.h"
#define PI 3.14159265
#define WHO_AM_I_MPU9250 0x75 // Should return 0x71

#define I2C_SLV0_REG     0x26

#define I2C_SLV1_REG     0x29

#define I2C_SLV2_REG     0x2C

#define I2C_SLV3_REG     0x2F

#define I2C_SLV4_REG     0x32

#define DMP_REG          0x6F  // Register in DMP from which to read or to which to write

#define  FIFO_R_W  0x74
#define PID_PARAM_KP        100            /* Proporcional */
#define PID_PARAM_KI        0.025        /* Integral */
#define PID_PARAM_KD        20            /* Derivative */

// Seven-bit device address is 110100 for ADO = 0
#define MPU9250_ADDRESS 0x68  // Device address when ADO = 0  dah howa slave address
void sendToLogger (double yaw , double pitch,double roll);
float REF = 180;
float temp;
void delay(uint16_t count)
{
  int i=0;
  while(i!=count)i++;
  
}
void I2C_init(void){

	    /*
         *Description
         *define varibles of GPIOx& I2Cx*/
	GPIO_InitTypeDef GPIO_InitStruct;
	I2C_InitTypeDef I2C_InitStruct;
        
       
     
        
        
	    /*
         *Description
         *extra to check i2c working*/
        GPIO_InitTypeDef GPIO_Output;     // For some debugging LEDs
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
 
	    /*
           *for timer
         *Description
         *Configure PD12, PD13, PD14 and PD15 in output pushpull mode */
        /* Alternating functions for pins */
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_TIM4);
    
        GPIO_Output.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15 ;
        GPIO_Output.GPIO_Mode = GPIO_Mode_AF;
        GPIO_Output.GPIO_OType = GPIO_OType_PP;
        GPIO_Output.GPIO_Speed = GPIO_Speed_100MHz;
        GPIO_Output.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_Init(GPIOD, &GPIO_Output);
        
        
        
        /*
         *Description
         *enable APB1 peripheral clock for I2Cx*/
	// enable APB1 peripheral clock for I2C1
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
      
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
       
       
	GPIO_InitStruct.GPIO_Pin =GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;			// set pins to alternate function
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;		        // set GPIO speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;		  	// set output to open drain --> the line has to be only pulled low, not driven high
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;			// enable pull up resistors
	GPIO_Init(GPIOB, &GPIO_InitStruct);			        // init GPIOB
      
	
       
       
        /*
         *Description
         *configure I2Cx*/
	// configure I2C1 
        // configure I2C1 
        // Connect I2C1 pins to AF  
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1);	// SCL for I2C1
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_I2C1); // SDA for I2C1
        
	I2C_InitStruct.I2C_ClockSpeed = 400000; 		// 400 kHz
	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;			// I2C mode
	I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;	        // 50% duty cycle --> standard
	I2C_InitStruct.I2C_OwnAddress1 = 0x00;			// own address, not relevant in master mode
	I2C_InitStruct.I2C_Ack = I2C_Ack_Disable;		// disable acknowledge when reading (can be changed later on)
	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; // set address length to 7 bit addresses
	I2C_Init(I2C1, &I2C_InitStruct);				// init I2C1
       
        /*
         *Description
         *enable I2Cx*/
	// enable I2C1
	I2C_Cmd(I2C1, ENABLE);
       
      
}

/*
 *Description
 *This function issues a start condition and 
 * transmits the slave address + R/W bit
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
		while(!I2C_CheckEvent(I2Cx,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
	}
}

void I2C_start2(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction){
	// wait until I2C1 is not busy any more
	//while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));
  
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
		while(!I2C_CheckEvent(I2Cx,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
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
	// wait for I2C1 EV8 --> last byte is still being transmitted (last byte in SR, buffer empty), next byte can already be writte
	I2C_SendData(I2Cx, data);
        while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTING));
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
	uint8_t data = (uint8_t)I2Cx->DR;
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
	while( !I2C_CheckEvent(I2Cx,  I2C_EVENT_MASTER_BYTE_RECEIVED) );
	// read data from I2C data register and return data byte
	uint8_t data =I2C_ReceiveData(I2Cx);
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
/*
 *Description
 *This function write bytes  from specific register address in imu
 */
void writeByte(uint8_t slave_address, uint8_t reg, uint8_t data)
{
	I2C_start(I2C1, slave_address<<1, I2C_Direction_Transmitter); // start a transmission in Master transmitter mode
        I2C_write(I2C1, reg);
	I2C_write(I2C1, data);              // Put data in Tx buffer
	I2C_stop(I2C1);           
}
/*
 *Description
 *This function read byte  from specific register address in imu
 */
uint8_t readByte(uint8_t slave_address , uint8_t reg)
{
        uint8_t received_data;
        I2C_start(I2C1,  slave_address<<1, I2C_Direction_Transmitter); // start a transmission in Master transmitter mode
        I2C_write(I2C1, reg);
        I2C_start2(I2C1, slave_address<<1, I2C_Direction_Receiver); // start a transmission in Master receiver mode
	received_data = I2C_read_nack(I2C1); // read one byte and don't request another byte, stop transmission
        return received_data;
}
/*
 *Description
 *This function read bytes  from specific register address in imu
 */
uint8_t * readBytes(uint8_t slave_address, uint8_t reg, uint8_t count, uint8_t * dest)
{  
	I2C_start(I2C1, slave_address<<1, I2C_Direction_Transmitter); // start a transmission in Master transmitter mode
        I2C_write(I2C1, reg);
        I2C_start2(I2C1, slave_address<<1, I2C_Direction_Receiver); // start a transmission in Master receiver mode
	uint8_t i = 0;
	while (i!=count) 
        {
          if(i==count-1)
            dest[i++] = I2C_read_nack(I2C1);  //read last value
           else
        dest[i++] = I2C_read_ack(I2C1);    // Put read results in the Rx buffer
        }         
       return dest;
}


//uart1 to to send to logger 
void init_USART1(uint32_t baudrate){
	
	/* This is a concept that has to do with the libraries provided by ST
	 * to make development easier the have made up something similar to 
	 * classes, called TypeDefs, which actually just define the common
	 * parameters that every peripheral needs to work correctly
	 * 
	 * They make our life easier because we don't have to mess around with 
	 * the low level stuff of setting bits in the correct registers
	 */
	GPIO_InitTypeDef GPIO_InitStruct; // this is for the GPIO pins used as TX and RX
	USART_InitTypeDef USART_InitStruct; // this is for the USART1 initilization
	
	
	/* enable APB2 peripheral clock for USART1 
	 * note that only USART1 and USART6 are connected to APB2
	 * the other USARTs are connected to APB1
	 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	
	/* enable the peripheral clock for the pins used by 
	 * USART1, PB6 for TX and PB7 for RX
	 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
	/* This sequence sets up the TX and RX pins 
	 * so they work correctly with the USART1 peripheral
	 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; // Pins 9 (TX) and 10 (RX) are used
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; 			// the pins are configured as alternate function so the USART peripheral has access to them
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;		// this defines the IO speed and has nothing to do with the baudrate!
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;			// this defines the output type as push pull mode (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;			// this activates the pullup resistors on the IO pins
	GPIO_Init(GPIOA, &GPIO_InitStruct);					// now all the values are passed to the GPIO_Init() function which sets the GPIO registers
	
	/* The RX and TX pins are now connected to their AF
	 * so that the USART1 can take over control of the 
	 * pins
	 */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1); 
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
	
	/* Now the USART_InitStruct is used to define the 
	 * properties of USART1 
	 */
	USART_InitStruct.USART_BaudRate = baudrate;				// the baudrate is set to the value we passed into this init function
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;// we want the data frame size to be 8 bits (standard)
	USART_InitStruct.USART_StopBits = USART_StopBits_1;		// we want 1 stop bit (standard)
	USART_InitStruct.USART_Parity = USART_Parity_No;		// we don't want a parity bit (standard)
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_RTS_CTS; // we don't want flow control (standard)
	USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; // we want to enable the transmitter and the receiver
	USART_Init(USART1, &USART_InitStruct);					// again all the properties are passed to the USART_Init function which takes care of all the bit setting
	
	
	
	// finally this enables the complete USART1 peripheral
	USART_Cmd(USART1, ENABLE);
}

 /*usart configuration*/
//to recive from AMR
void init_USART3(uint32_t baudrate){
	
	/* This is a concept that has to do with the libraries provided by ST
	 * to make development easier the have made up something similar to 
	 * classes, called TypeDefs, which actually just define the common
	 * parameters that every peripheral needs to work correctly
	 * 
	 * They make our life easier because we don't have to mess around with 
	 * the low level stuff of setting bits in the correct registers
	 */
	GPIO_InitTypeDef GPIO_InitStruct; // this is for the GPIO pins used as TX and RX
	USART_InitTypeDef USART_InitStruct; // this is for the USART3 initilization
	NVIC_InitTypeDef NVIC_InitStructure; // this is used to configure the NVIC (nested vector interrupt controller)
        
        
	/* enable APB1 peripheral clock for USART3
	 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	
	/* enable the peripheral clock for the pins used by 
	 * USART3, PB10 for TX and PB11 for RX
	 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
	/* This sequence sets up the TX and RX pins 
	 * so they work correctly with the USART3 peripheral
	 */
	GPIO_InitStruct.GPIO_Pin =  GPIO_Pin_10|GPIO_Pin_11; // Pins 10 (TX) and 11 (RX) are used ,pin 12 for (CK)
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; 			// the pins are configured as alternate function so the USART peripheral has access to them
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;		// this defines the IO speed and has nothing to do with the baudrate!
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;			// this defines the output type as push pull mode (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;			// this activates the pullup resistors on the IO pins
	GPIO_Init(GPIOB, &GPIO_InitStruct);   // now all the values are passed to the GPIO_Init() function which sets the GPIO registers
	
        //new at 26/6
        //PA10 = USART1.RX => Input  dah law ha receive mn Amr
        /*GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
        GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;		// this defines the IO speed and has nothing to do with the baudrate!
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;			// this defines the output type as push pull mode (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
        GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
        GPIO_Init(GPIOB, &GPIO_InitStruct);
	/* The RX and TX pins are now connected to their AF
	 * so that the USART3 can take over control of the 
	 * pins
	 */
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);   //Tx
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);  //Rx
        
	
	/* Now the USART_InitStruct is used to define the 
	 * properties of USART3
	 */
	USART_InitStruct.USART_BaudRate = baudrate;				// the baudrate is set to the value we passed into this init function
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;// we want the data frame size to be 8 bits (standard)
	USART_InitStruct.USART_StopBits = USART_StopBits_1;		// we want 1 stop bit (standard)
	USART_InitStruct.USART_Parity = USART_Parity_No;		// we don't want a parity bit (standard)  (old)
	//USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // we don't want flow control (standard)  (old)
        USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_RTS_CTS;   //new at 19/6 to make flow control feh types tanya like CTS and RTS_CTS  momkn agrbha
	USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; // we want to enable the transmitter and the receiver
	USART_Init(USART3, &USART_InitStruct);					// again all the properties are passed to the USART_Init function which takes care of all the bit setting

       
	// finally this enables the complete USART3 peripheral
	USART_Cmd(USART3, ENABLE);
}
/*to send string */
void USART_puts(USART_TypeDef* USARTx, volatile char *s){

	while(*s)
        {
		USART_SendData(USARTx, *s);
		*s++;
                // wait until data register is empty
		while( !(USARTx->SR & 0x00000040) ); 
	}
        
}
/*to send one character using uart*/
void UU_PutChar(USART_TypeDef* USARTx, uint8_t ch)
{
  while(!(USARTx->SR & USART_SR_TXE));
  USARTx->DR = ch;  
}
/*to send one character using uart*/
uint8_t UU_recivetChar(USART_TypeDef* USARTx)
{
  while(!(USART_GetFlagStatus(USART3, USART_FLAG_RXNE)));
  uint8_t ch=USARTx->DR ; 
  return ch;
}

//USART_FLAG_RXNE: Receive data register not empty flag
/*to read one character using uart*/
uint8_t * readBytesU(USART_TypeDef* USARTx,uint8_t * dest,uint8_t count)
{
 for(uint8_t i=0;i<count;i++)
        {
         dest[i] = UU_recivetChar(USARTx);
        }   
 return dest;
}
//another function to receive string
char* usart3_RecieveString(char* String)
{ 
while(*String != 0)
{ 
while(USART_GetFlagStatus(USART3,USART_FLAG_RXNE) == RESET); 
*String++ = USART_ReceiveData(USART3); 
} 
return String; 
}


void UU_PutString(USART_TypeDef* USARTx, uint8_t * str)
{
  while(*str != 0)
  {
    UU_PutChar(USARTx, *str);
    str++;
  }
}

double pitch,yaw,roll;
/////////////////////////////////////////////////////

short concatenate(uint8_t x, uint8_t y) {
  if (y==0x99) return x;
    uint8_t pow = 10;
    while(y >= pow)
        pow *= 10;
    return x * pow + y;        
}

//////////////////////////////////////////////////////////


float to_motor_angles (float imu_angle)
{
  float temp =  5*imu_angle+640;
  while (temp < 640) temp+=640;
  while (temp>2440) temp-=640;
  return   temp;
}

void initializeMotor()
{    // Structures for configuration
    GPIO_InitTypeDef            GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef     TIM_TimeBaseStructure;
    TIM_OCInitTypeDef           TIM_OCInitStructure;

    // TIM3 Clock Enable
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    // GPIOA Clock Enable
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    // GPIOB Clock Enable
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    // Initalize PA6 (TIM3 Ch1) and PA7 (TIM3 Ch2)
    GPIO_InitStructure.GPIO_Pin     = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_100MHz;    // GPIO_High_Speed
    GPIO_InitStructure.GPIO_OType   = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd    = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // Initalize PB0 (TIM3 Ch3)
       GPIO_InitStructure.GPIO_Pin     = GPIO_Pin_0;
       GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_AF;
       GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_100MHz;    // GPIO_High_Speed
       GPIO_InitStructure.GPIO_OType   = GPIO_OType_PP;
       GPIO_InitStructure.GPIO_PuPd    = GPIO_PuPd_UP;
       GPIO_Init(GPIOB, &GPIO_InitStructure);

    // Assign Alternate Functions to pins
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_TIM3);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_TIM3);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM3);




    // Time Base Configuration
    TIM_TimeBaseStructure.TIM_Period        = 19999;
    TIM_TimeBaseStructure.TIM_Prescaler     = 84;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;

    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    // Common TIM Settings
    TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse       = 0;                        // Initial duty cycle
    TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;

    // Channel 1
    TIM_OC1Init(TIM3, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

    // Channel 2
    TIM_OC2Init(TIM3, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);

    // Channel 3
    TIM_OC3Init(TIM3, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM3, ENABLE);

    // Start timer
    TIM_Cmd(TIM3, ENABLE);


}



void SetSysTick(void){
	// ---------- SysTick timer (1ms) -------- //
	if (SysTick_Config(SystemCoreClock / 1000)) {
		// Capture error
		while (1){};
	}
}


void SetRoll(unsigned char roll)
{
  if (roll<=90)
	{
	TIM3->CCR1=640+(roll*9);
	}
	else
	{
		TIM3->CCR1=640+(roll*10);
	}
}
void SetPitch(unsigned char pitch)
{
	if (pitch<90)
	{
	TIM3->CCR2=250+(pitch*14);
	}
	else if (pitch>=90)
	{
		TIM3->CCR2=260+(pitch*14);
	}
	else
	{
		//do nothing ;
	}
}



void SetYaw(unsigned char yaw)
{
	if (yaw>=0 && yaw<20)
	{
		TIM3->CCR3=1080+(yaw*5);
	}
	else if (yaw>=20 && yaw<79)
	{
		TIM3->CCR3=1075+(yaw*5);
	}
	else if (yaw>=80 && yaw<110)
	{
		TIM3->CCR3=1070+(yaw*5);
	}

	else if (yaw>=110 && yaw<=150)
	{
		TIM3->CCR3=1075+(yaw*5);
	}
	else if (yaw>150 && yaw<=170)
	{
		TIM3->CCR3=1080+(yaw*5);
	}
	else
	{
		//do nothing
	}

}
//the actual yaw takes from 0 to 170 , but it's needed from 0 to 360 for unification
float yaw_in_170_range ( float in_360_range)
{
return 0.47223*in_360_range;

}
//the actual pitch takes from 0 to 180 , but it's needed from 0 to 360 for unification
float pitch_in_180_range ( float in_360_range)
{
return 0.5*in_360_range;

}
/*
*to read data from mpu6050
*/
s16 AccelGyro[6];
s16 AccelGyro2[6];
double acclX_scaled, acclY_scaled, acclZ_scaled;
double gyroX_scaled, gyroY_scaled, gyroZ_scaled;
#define M_PI 3.14
double dist(double a, double b)
{
  return sqrt((a*a) + (b*b));
}
//range from -90 to 90
double get_y_rotation(double x, double y, double z)
{
  double radians;
  
  radians = atan2(x, dist(y, z));
  return -(radians * (180.0 / M_PI));
}
//range from -90 to 90
double get_x_rotation(double x, double y, double z)
{
  double radians;
  radians = atan2(y, dist(x, z));
  return (radians * (180.0 / M_PI));
}
//range from 0 to 360
double get_z_rotation(double x, double y, double z)
{
  double radians;
  radians = atan2(x,z);
  return (radians * (180.0 / M_PI));
}
float avg (float *arr)
{
float temp=0;
int length = 1000;
for ( int i =0;i<length;i++)
temp+=arr[i];
return temp/length;
}
void lanes_mode ()
{
  
  //let the  user choose ref value due to the application 
float initial_pitch=pitch_in_180_range(REF);
float initial_yaw=yaw_in_170_range(REF);
  SetPitch(initial_pitch);
  SetRoll(85);
 
  //old for yaw for right anf left 
  float old_value =0.0;
  float old_velocity=0.0;
  float old_acc=NULL ;
  //old pitch for PD controller 
  float old_pitch =NULL;
        
        
  //initialize imu    
  I2C_init();
  uint8_t p2=readByte(0x68 , 0x75);
  MPU6050_Initialize(); 
     
 /*for control 
   Last Error = Error

Error = Set Point – Process Variable

Derivative = Error – Last Error

Control Variable = (Kp * Error) + (Kd * Derivative) 
   define PID_PARAM_KP        1             
#define PID_PARAM_KD        2            
   */
   
   
   int incremental = 0;
   float old_pitch_arr [1000]=0;
     while(1) 
{
    if (incremental>1000) incremental=0;
    incremental++;
    MPU6050_GetRawAccelGyro( AccelGyro);
    pitch=get_x_rotation( AccelGyro[0] / 16384.0,  AccelGyro[1] / 16384.0, AccelGyro[2]/ 16384.0);
    yaw =get_y_rotation( AccelGyro[0] / 16384.0,  AccelGyro[1] / 16384.0, AccelGyro[2]/ 16384.0);
    roll =get_z_rotation( AccelGyro[0] / 16384.0,  AccelGyro[1] / 16384.0, AccelGyro[2]/ 16384.0);
    sendToLogger(yaw,pitch,roll);

    //mae the pitch range from 0 to 360
     if(pitch<0)pitch=pitch+360.0;
     if (pitch<100&&pitch>90)pitch=90;
     if (pitch>=270)pitch-=360;       
     pitch+=90;
     pitch*=2;
     old_pitch_arr[incremental]=pitch;
     
      //mae the yaw range from 0 to 360
     if(yaw<0)yaw=yaw+360.0;
     if (yaw<100&&yaw>90)yaw=90;
     if (yaw>=270)yaw-=360;       
     yaw+=90;
     yaw*=2;
         
     memset(AccelGyro, 0, 12);
   //  delay(1333);  //100 ms 
     delay (1333);
     //save the obtained in new var
     float new_value = yaw;
     float new_pitch =pitch;
     float new_velocity =new_value -old_value;
     float new_acc;
     //make acceleration for stop rotating yaw if less then specific value
     if (old_acc!=NULL)new_acc= new_velocity-old_velocity;
     else  new_acc=0;
      
  
 if (new_acc>30)
    // if (1)
  {
      
    //yaw controlling 
     if (yaw>REF) 
      {
       float difference =yaw-REF;
        temp = REF-difference;
     //  SetYaw(yaw_in_170_range(temp));
      }
 
     if (yaw<REF) 
      {
      float difference =REF-yaw;
       temp = REF+difference;
    //  SetYaw(yaw_in_170_range(temp));
      } 
   
     if (yaw ==REF);
     //  SetYaw(initial_yaw);
  }
     
     old_value=new_value;
     old_velocity=new_velocity;
     old_acc=new_acc;
  
  
  //pitch controlling
  
        if (old_pitch==NULL)
     {
       if (pitch>REF) 
      {
       float difference =pitch-REF;
        temp = REF-difference;
        if (temp>170&&temp<190)
       SetPitch(pitch_in_180_range(temp));

        
        
        
      }
        else if (pitch<REF) 
      {
       float difference =REF-pitch;
        temp = REF+difference;
        if (temp>170&&temp<190)
       SetPitch(pitch_in_180_range(temp));
     
      } 

     else if (pitch ==REF)
      
     {  SetPitch(initial_pitch);  
     }
     }
     // to eliminate vibrations 
   else   if ((new_pitch-old_pitch>3||old_pitch-new_pitch>3))
  //  else if (avg(old_pitch_arr)-pitch>1||pitch-avg(old_pitch_arr)>1)
     {
     
      if (pitch>REF) 
      {
       float difference =pitch-REF;
        temp = REF-difference;
         if (temp>140&&temp<220)
        SetPitch(pitch_in_180_range(temp));

      }

     else if (pitch<REF) 
      {
         float difference =REF-pitch;
       temp = REF+difference;
      if (temp>140&&temp<220)
      SetPitch(pitch_in_180_range(temp));

      }
   else  if (pitch ==REF)
   { 
     
         SetPitch(initial_pitch);
        
   }
     }
old_pitch=pitch;
}
}
  
  

void stabilize_mode()
{

                  
//let the  user choose ref value due to the application 
float initial_pitch=pitch_in_180_range(REF);
float initial_yaw=yaw_in_170_range(REF);
  SetPitch(initial_pitch);
  SetRoll(85);
 
  //old for yaw for right anf left 
  float old_value =0.0;
  float old_velocity=0.0;
  float old_acc=NULL ;
  //old pitch for PD controller 
  float old_pitch =NULL;
        
        
  //initialize imu    
  I2C_init();
  uint8_t p2=readByte(0x68 , 0x75);
  MPU6050_Initialize(); 
     
 /*for control 
   Last Error = Error

Error = Set Point – Process Variable

Derivative = Error – Last Error

Control Variable = (Kp * Error) + (Kd * Derivative) 
   define PID_PARAM_KP        1             
#define PID_PARAM_KD        2            
   */
   
   
   int incremental = 0;
   float old_pitch_arr [1000]=0;
     while(1) 
{
    if (incremental>1000) incremental=0;
    incremental++;
    MPU6050_GetRawAccelGyro( AccelGyro);
    pitch=get_x_rotation( AccelGyro[0] / 16384.0,  AccelGyro[1] / 16384.0, AccelGyro[2]/ 16384.0);
    yaw =get_y_rotation( AccelGyro[0] / 16384.0,  AccelGyro[1] / 16384.0, AccelGyro[2]/ 16384.0);
    roll =get_z_rotation( AccelGyro[0] / 16384.0,  AccelGyro[1] / 16384.0, AccelGyro[2]/ 16384.0);
    sendToLogger(yaw,pitch,roll);

    //mae the pitch range from 0 to 360
     if(pitch<0)pitch=pitch+360.0;
     if (pitch<100&&pitch>90)pitch=90;
     if (pitch>=270)pitch-=360;       
     pitch+=90;
     pitch*=2;
     old_pitch_arr[incremental]=pitch;
     
      //mae the yaw range from 0 to 360
     if(yaw<0)yaw=yaw+360.0;
     if (yaw<100&&yaw>90)yaw=90;
     if (yaw>=270)yaw-=360;       
     yaw+=90;
     yaw*=2;
         
     memset(AccelGyro, 0, 12);
   //  delay(1333);  //100 ms 
     delay (200000000000000000);
     //save the obtained in new var
     float new_value = yaw;
     float new_pitch =pitch;
     float new_velocity =new_value -old_value;
     float new_acc;
     //make acceleration for stop rotating yaw if less then specific value
     if (old_acc!=NULL)new_acc= new_velocity-old_velocity;
     else  new_acc=0;
      
  
 if (new_acc>30)
    // if (1)
  {
      
    //yaw controlling 
     if (yaw>REF) 
      {
       float difference =yaw-REF;
        temp = REF-difference;
     //  SetYaw(yaw_in_170_range(temp));
      }
 
     if (yaw<REF) 
      {
      float difference =REF-yaw;
       temp = REF+difference;
     // SetYaw(yaw_in_170_range(temp));
      } 
   
     if (yaw ==REF);
     //  SetYaw(initial_yaw);
  }
     
     old_value=new_value;
     old_velocity=new_velocity;
     old_acc=new_acc;
  
  
  //pitch controlling
  
        if (old_pitch==NULL)
     {
       if (pitch>REF) 
      {
       float difference =pitch-REF;
        temp = REF-difference;
       SetPitch(pitch_in_180_range(temp));

        
        
        
      }
        else if (pitch<REF) 
      {
       float difference =REF-pitch;
        temp = REF+difference;
       SetPitch(pitch_in_180_range(temp));
     
      } 

     else if (pitch ==REF)
      
     {  SetPitch(initial_pitch);  
     }
     }
     // to eliminate vibrations 
   //   else   if ((new_pitch-old_pitch>3||old_pitch-new_pitch>3))
    else if (avg(old_pitch_arr)-pitch>1||pitch-avg(old_pitch_arr)>1)
     {
     
      if (pitch>REF) 
      {
       float difference =pitch-REF;
        temp = REF-difference;

        SetPitch(pitch_in_180_range(temp));

      }

     else if (pitch<REF) 
      {
         float difference =REF-pitch;
       temp = REF+difference;
     
      SetPitch(pitch_in_180_range(temp));

      }
   else  if (pitch ==REF)
   { 
         SetPitch(initial_pitch);
        
   }
     }
old_pitch=pitch;
}
}

char point='.';
char fasla=',';
char end='\n';
char pitch_total[7]=0;
char yaw_total[7]=0;

char roll_total[7]=0;
float tmpVal ,tmpValP,tmpValR;
char concatenateToaa[21]=0;   //to concatenate yaw and pitch till nw 

/*
*convert from decimal to string
*/
void dec_to_str (char* str, uint32_t val, size_t digits)
{
  size_t i=1u;

  for(; i<=digits; i++)
  {
    str[digits-i] = (char)((val % 10u) + '0');
    val/=10u;
  }

  //str[i-1u] = '\0'; // assuming you want null terminated strings
}
void sendToLogger (double yaw , double pitch,double roll)
{
     init_USART3(9600);  //to send to logger 
  /*convert yaw to string*/
     tmpVal = (yaw< 0) ? -yaw : yaw;
     int y1UART = tmpVal;                  // Get the integer (678).
     float tmpFrac = tmpVal - y1UART;      // Get fraction (0.0123).
     int y2UART = trunc(tmpFrac * 1000);  // Turn into integer (123).
     char yaw1[3u + 1u]=0; // assuming you want null terminated strings?
     dec_to_str(yaw1, y1UART, 3u);
     char yaw2[3u + 1u]=0; // assuming you want null terminated strings?
     dec_to_str(yaw2, y2UART, 3u);
     /*convert pitch to string*/
     tmpValP = (pitch< 0) ? -pitch : pitch;
     int p1UART = tmpValP;                  // Get the integer (678).
     float tmpFracP = tmpValP - p1UART;      // Get fraction (0.0123).
     int p2UART = trunc(tmpFracP * 1000);  // Turn into integer (123).
     char pitch1[3u + 1u]=0; // assuming you want null terminated strings?
     dec_to_str(pitch1, p1UART, 3u);
     char pitch2[3u + 1u]=0; // assuming you want null terminated strings?
     dec_to_str(pitch2, p2UART, 3u);
     /*convert roll to string*/
     tmpValR = (roll< 0) ? -roll : roll;
     int r1UART = tmpValR;                  // Get the integer (678).
     float tmpFracR = tmpValR - r1UART;      // Get fraction (0.0123).
     int r2UART = trunc(tmpFracR * 1000);  // Turn into integer (123).
     char roll1[3u + 1u]=0; // assuming you want null terminated strings?
     dec_to_str(roll1, r1UART, 3u);
     char roll2[3u + 1u]=0; // assuming you want null terminated strings?
     dec_to_str(roll2, r2UART, 3u);
     
     
     
     
     /*concatenate yaw in string*/
     for (int i =0;i<3;i++)
     yaw_total[i]=yaw1[i];
     yaw_total[3]='.';
     for (int i =4;i<6;i++)
     yaw_total[i]=yaw2[i-4];
     yaw_total[6]=','; //assumin en el yaw awel 7aga hb3tha fa h7ot b3dha fasla
     /*concatenate pitch in string*/
      for (int i =0;i<3;i++)
      pitch_total[i]=pitch1[i];
      pitch_total[3]='.';
      for (int i =4;i<6;i++)
      pitch_total[i]=pitch2[i-4];
      pitch_total[6]=',';  //assuming in el pitch a5er 7aga hb3tha fa h7ot \n 3shan nbd2 new line
      /*concatenate roll in string*/
      for (int i =0;i<3;i++)
      roll_total[i]=roll1[i];
      roll_total[3]='.';
      for (int i =4;i<6;i++)
      roll_total[i]=roll2[i-4];
      roll_total[6]='\n';  //assuming in el pitch a5er 7aga hb3tha fa h7ot \n 3shan nbd2 new line
      
      
      
      /*put yaw and pitch in array of string*/
      for (int i =0;i<7;i++)
      concatenateToaa[i]=yaw_total[i];
      for ( int i=7;i<14;i++)
      concatenateToaa[i]=pitch_total[i-7];
       //printf("%c",concatenateToaa);
       for ( int i=14;i<21;i++)
      concatenateToaa[i]=roll_total[i-14];
     // concatenateToaa[20]='\n';
    USART_puts(USART3, concatenateToaa);
    delay(90000);

}
/*Reciving from  Amrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrr**************************************/
uint8_t dest[40];
unsigned char x[4];
float Recive_Amr[10];
uint8_t i=0;
float yaw_Amr;
float pitch_Amr;
float roll_Amr;
float positin_N;
float positin_E;
float positin_D;
float velocity_N;
float velocity_E;
float velocity_D;
float time_Amr;
void recive_Amr()
{
init_USART3(9600);
readBytesU(USART3,dest,40);
for(uint8_t u=0;u<10;u++)
{
 for(uint8_t k=i;k<i+4;k++)
{
  x[k % 4]=dest[k];
  //printf("%02x\n",dest[k]);
}
Recive_Amr[u]=*(float*)&x;
printf("%f  amr\n",Recive_Amr[u]);
i=i+4;
   }
float yaw_Amr=Recive_Amr[0];
float pitch_Amr=Recive_Amr[1];
float roll_Amr=Recive_Amr[2];
float positin_N=Recive_Amr[3];
float positin_E=Recive_Amr[4];
float positin_D=Recive_Amr[5];
float velocity_N=Recive_Amr[6];
float velocity_E=Recive_Amr[7];
float velocity_D=Recive_Amr[8];
float time_Amr=Recive_Amr[9];
   
}

int main()
{
  volatile int * i;
 /* float arr[10]={1,2,3,4,5,6,7,8,9,10};
  float average = avg (arr);
   printf("Average of array values is %.2f", average);*/
  init_USART3(9600);
    USART_puts(USART3, "gfhfhgfhgfhgdhgtdjy\n");
  sendToLogger (120.5,50.65,1.55);
   SetSysTick();
	//config_PWM();

    /* Create keypad instance */
    TM_KEYPAD_Button_t Keypad_Button;

    /* Initialize system */
    SystemInit();

    /* Init delay functions */
    TM_DELAY_Init();

    /* Initialize leds */
    TM_DISCO_LedInit();

    initializeMotor();
  
    TM_KEYPAD_Init(TM_KEYPAD_Type_Large);
 /*while(1)
  {
    recive_Amr();
  }*/
  /*I2C_init();
  uint8_t p2=readByte(0x68 , 0x75);
  MPU6050_Initialize(); */
/*SetYaw(pitch_in_180_range(180));
SetYaw(pitch_in_180_range(190));
 SetYaw(pitch_in_180_range(170));
 SetYaw(pitch_in_180_range(200));*/
 
  while(1)
  {// lanes_mode();  
 stabilize_mode();
   MPU6050_GetRawAccelGyro( AccelGyro);
    pitch=get_x_rotation( AccelGyro[0] / 16384.0,  AccelGyro[1] / 16384.0, AccelGyro[2]/ 16384.0);
    yaw=get_y_rotation( AccelGyro[0] / 16384.0,  AccelGyro[1] / 16384.0, AccelGyro[2]/ 16384.0);
    roll = get_z_rotation( AccelGyro[0] / 16384.0,  AccelGyro[1] / 16384.0, AccelGyro[2]/ 16384.0);
     if(pitch<0)pitch=pitch+360.0;
     if(yaw<0)yaw=yaw+360.0;
     memset(AccelGyro, 0, 12);
     delay(1333);  //100 ms 
     
     
      /* Read keyboard data */
        Keypad_Button = TM_KEYPAD_Read();

        /* Keypad was pressed */
        if (Keypad_Button != TM_KEYPAD_Button_NOPRESSED) 
        {/* Keypad is pressed */
            switch (Keypad_Button) 
            {
                case TM_KEYPAD_Button_0:        /* Button 0 pressed */
                  REF=180;
                  stabilize_mode();
                    
                    break;
                case TM_KEYPAD_Button_1:        /* Button 1 pressed */
                    REF=180;
                    lanes_mode();
                                 
                case TM_KEYPAD_Button_2:        /* Button 2 pressed */           
                 SetPitch(pitch_in_180_range(180));
                 //SetYaw(yaw_in_170_range(180));
                 sendToLogger(yaw,pitch,roll);
                    break;
            }     
        }
  }
     
  return 0;
}

/* 1ms handler */
void TM_DELAY_1msHandler(void)
{
    /* Process keypad */
    TM_KEYPAD_Update();
}