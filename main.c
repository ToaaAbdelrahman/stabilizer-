#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include <stm32f4xx_i2c.h>
#include <stm32f4xx_tim.h>
//#include "pitch_stabilize.h>
#include <stdio.h>

#include <math.h>
void Delay(__IO uint32_t time);
extern  __IO uint32_t timingDelay;
float to_motor_angles (float imu_angle);

/*mpu9250 register*/
//Magnetometer Registers
#define AK8963_ADDRESS   0x0C
#define AK8963_WHO_AM_I  0x00 // should return 0x48
#define AK8963_INFO      0x01
#define AK8963_ST1       0x02  // data ready status bit 0
#define AK8963_XOUT_L	 0x03  // data
#define AK8963_XOUT_H	 0x04
#define AK8963_YOUT_L	 0x05
#define AK8963_YOUT_H	 0x06
#define AK8963_ZOUT_L	 0x07
#define AK8963_ZOUT_H	 0x08
#define AK8963_ST2       0x09  // Data overflow bit 3 and data read error status bit 2
#define AK8963_CNTL      0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_ASTC      0x0C  // Self test control
#define AK8963_I2CDIS    0x0F  // I2C disable
#define AK8963_ASAX      0x10  // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY      0x11  // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ      0x12  // Fuse ROM z-axis sensitivity adjustment value

#define SELF_TEST_X_GYRO 0x00                  
#define SELF_TEST_Y_GYRO 0x01                                                                          
#define SELF_TEST_Z_GYRO 0x02

#define SELF_TEST_X_ACCEL 0x0D
#define SELF_TEST_Y_ACCEL 0x0E    
#define SELF_TEST_Z_ACCEL 0x0F

#define SELF_TEST_A      0x10

#define XG_OFFSET_H      0x13  // User-defined trim values for gyroscope
#define XG_OFFSET_L      0x14
#define YG_OFFSET_H      0x15
#define YG_OFFSET_L      0x16
#define ZG_OFFSET_H      0x17
#define ZG_OFFSET_L      0x18
#define SMPLRT_DIV       0x19
#define CONFIG           0x1A
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define ACCEL_CONFIG2    0x1D
#define LP_ACCEL_ODR     0x1E   
#define WOM_THR          0x1F   

#define MOT_DUR          0x20  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define ZMOT_THR         0x21  // Zero-motion detection threshold bits [7:0]
#define ZRMOT_DUR        0x22  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms

#define FIFO_EN          0x23
#define I2C_MST_CTRL     0x24   
#define I2C_SLV0_ADDR    0x25
#define I2C_SLV0_REG     0x26
#define I2C_SLV0_CTRL    0x27
#define I2C_SLV1_ADDR    0x28
#define I2C_SLV1_REG     0x29
#define I2C_SLV1_CTRL    0x2A
#define I2C_SLV2_ADDR    0x2B
#define I2C_SLV2_REG     0x2C
#define I2C_SLV2_CTRL    0x2D
#define I2C_SLV3_ADDR    0x2E
#define I2C_SLV3_REG     0x2F
#define I2C_SLV3_CTRL    0x30
#define I2C_SLV4_ADDR    0x31
#define I2C_SLV4_REG     0x32
#define I2C_SLV4_DO      0x33
#define I2C_SLV4_CTRL    0x34
#define I2C_SLV4_DI      0x35
#define I2C_MST_STATUS   0x36
#define INT_PIN_CFG      0x37
#define INT_ENABLE       0x38
#define DMP_INT_STATUS   0x39  // Check DMP interrupt
#define INT_STATUS       0x3A
#define ACCEL_XOUT_H     0x3B
#define ACCEL_XOUT_L     0x3C
#define ACCEL_YOUT_H     0x3D
#define ACCEL_YOUT_L     0x3E
#define ACCEL_ZOUT_H     0x3F
#define ACCEL_ZOUT_L     0x40
#define TEMP_OUT_H       0x41
#define TEMP_OUT_L       0x42
#define GYRO_XOUT_H      0x43
#define GYRO_XOUT_L      0x44
#define GYRO_YOUT_H      0x45
#define GYRO_YOUT_L      0x46
#define GYRO_ZOUT_H      0x47
#define GYRO_ZOUT_L      0x48
#define EXT_SENS_DATA_00 0x49
#define EXT_SENS_DATA_01 0x4A
#define EXT_SENS_DATA_02 0x4B
#define EXT_SENS_DATA_03 0x4C
#define EXT_SENS_DATA_04 0x4D
#define EXT_SENS_DATA_05 0x4E
#define EXT_SENS_DATA_06 0x4F
#define EXT_SENS_DATA_07 0x50
#define EXT_SENS_DATA_08 0x51
#define EXT_SENS_DATA_09 0x52
#define EXT_SENS_DATA_10 0x53
#define EXT_SENS_DATA_11 0x54
#define EXT_SENS_DATA_12 0x55
#define EXT_SENS_DATA_13 0x56
#define EXT_SENS_DATA_14 0x57
#define EXT_SENS_DATA_15 0x58
#define EXT_SENS_DATA_16 0x59
#define EXT_SENS_DATA_17 0x5A
#define EXT_SENS_DATA_18 0x5B
#define EXT_SENS_DATA_19 0x5C
#define EXT_SENS_DATA_20 0x5D
#define EXT_SENS_DATA_21 0x5E
#define EXT_SENS_DATA_22 0x5F
#define EXT_SENS_DATA_23 0x60
#define MOT_DETECT_STATUS 0x61
#define I2C_SLV0_DO      0x63
#define I2C_SLV1_DO      0x64
#define I2C_SLV2_DO      0x65
#define I2C_SLV3_DO      0x66
#define I2C_MST_DELAY_CTRL 0x67
#define SIGNAL_PATH_RESET  0x68
#define MOT_DETECT_CTRL  0x69
#define USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2       0x6C
#define DMP_BANK         0x6D  // Activates a specific bank in the DMP
#define DMP_RW_PNT       0x6E  // Set read/write pointer to a specific start address in specified DMP bank
#define DMP_REG          0x6F  // Register in DMP from which to read or to which to write
#define DMP_REG_1        0x70
#define DMP_REG_2        0x71 
#define FIFO_COUNTH      0x72
#define FIFO_COUNTL      0x73
#define FIFO_R_W         0x74
#define WHO_AM_I_MPU9250 0x75 // Should return 0x71
#define XA_OFFSET_H      0x77
#define XA_OFFSET_L      0x78
#define YA_OFFSET_H      0x7A
#define YA_OFFSET_L      0x7B
#define ZA_OFFSET_H      0x7D
#define ZA_OFFSET_L      0x7E

// Seven-bit device address is 110100 for ADO = 0
#define MPU9250_ADDRESS 0x68  // Device address when ADO = 0  dah howa slave address
#define AK8963_ADDRESS 0x0C //  Address of magnetometer
#define PI 3.14

void delay(uint8_t count)
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
        I2C_InitTypeDef I2C_InitStruct2;
       
     
        
        
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
       
	GPIO_InitStruct.GPIO_Pin =GPIO_Pin_6 | GPIO_Pin_7| GPIO_Pin_10|GPIO_Pin_11;
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
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_I2C1); // SDA for I2C1
        // Connect I2C2 pins to AF  
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_I2C2);   //scl for I2C2
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_I2C2);  //sDa for I2C2
       
       
        /*
         *Description
         *configure I2Cx*/
	// configure I2C1 
        // configure I2C1 
	I2C_InitStruct.I2C_ClockSpeed = 400000; 		// 400 kHz
	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;			// I2C mode
	I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;	        // 50% duty cycle --> standard
	I2C_InitStruct.I2C_OwnAddress1 = 0x00;			// own address, not relevant in master mode
	I2C_InitStruct.I2C_Ack = I2C_Ack_Disable;		// disable acknowledge when reading (can be changed later on)
	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; // set address length to 7 bit addresses
	I2C_Init(I2C1, &I2C_InitStruct);				// init I2C1
        // configure I2C2
        I2C_InitStruct2.I2C_ClockSpeed = 400000; 		// 400 kHz
	I2C_InitStruct2.I2C_Mode = I2C_Mode_I2C;			// I2C mode
	I2C_InitStruct2.I2C_DutyCycle = I2C_DutyCycle_2;	        // 50% duty cycle --> standard
	I2C_InitStruct2.I2C_OwnAddress1 = 0x00;			// own address, not relevant in master mode
	I2C_InitStruct2.I2C_Ack = I2C_Ack_Disable;		// disable acknowledge when reading (can be changed later on)
	I2C_InitStruct2.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; // set address length to 7 bit addresses
	I2C_Init(I2C2, &I2C_InitStruct2);				// init I2C2
     
	
        /*
         *Description
         *enable I2Cx*/
	// enable I2C1
	I2C_Cmd(I2C1, ENABLE);
        // enable I2C2
	I2C_Cmd(I2C2, ENABLE);
      
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


/*
 *Description
 *for mpu9250 configuration
===================================================================================================================
====== Set of useful function to access acceleration. gyroscope and magnetometer data
===================================================================================================================
*/
// Set initial input parameters
enum Ascale {
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
};

enum Gscale {
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
};

enum Mscale {
  MFS_14BITS = 0, // 0.6 mG per LSB
  MFS_16BITS      // 0.15 mG per LSB
};
// Specify sensor full scale
uint8_t Gscale = GFS_250DPS;
uint8_t Ascale = AFS_2G;
uint8_t Mscale = MFS_16BITS; // Choose either 14-bit or 16-bit magnetometer resolution
uint8_t Mmode = 0x02;        // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
float aRes, gRes, mRes; // scale resolutions per LSB for the sensors
int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output
float magCalibration[3] = {0, 0, 0}, magbias[3] = {0, 0, 0};  // Factory mag calibration and mag bias
float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0};      // Bias corrections for gyro and accelerometer


// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
float GyroMeasError = PI * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
float GyroMeasDrift = PI * (0.0f / 180.0f); // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)

#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

float pitch, yaw, roll;
float deltat = 0.0f, sum = 0.0f;        // integration interval for both filter schemes
uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
uint32_t Now = 0;        // used to calculate integration interval

float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values 
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f}; // vector to hold integral error for Mahony method


void getMres() {
  switch (Mscale)
  {
 	// Possible magnetometer scales (and their register bit settings) are:
	// 14 bit resolution (0) and 16 bit resolution (1)
    case MFS_14BITS:
          mRes = 10.*4912./8190.; // Proper scale to return milliGauss
          break;
    case MFS_16BITS:
          mRes = 10.*4912./32760.0; // Proper scale to return milliGauss
          break;
  }
}

void getGres() {
  switch (Gscale)
  {
 	// Possible gyro scales (and their register bit settings) are:
	// 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11). 
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case GFS_250DPS:
          gRes = 250.0/32768.0;
          break;
    case GFS_500DPS:
          gRes = 500.0/32768.0;
          break;
    case GFS_1000DPS:
          gRes = 1000.0/32768.0;
          break;
    case GFS_2000DPS:
          gRes = 2000.0/32768.0;
          break;
  }
}

void getAres() {
  switch (Ascale)
  {
 	// Possible accelerometer scales (and their register bit settings) are:
	// 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11). 
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case AFS_2G:
          aRes = 2.0/32768.0;
          break;
    case AFS_4G:
          aRes = 4.0/32768.0;
          break;
    case AFS_8G:
          aRes = 8.0/32768.0;
          break;
    case AFS_16G:
          aRes = 16.0/32768.0;
          break;
  }
}


void readAccelData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;  
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ; 
}


void readGyroData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;  
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ; 
}

void readMagData(int16_t * destination)
{
  uint8_t rawData[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
  if(readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01) { // wait for magnetometer data ready bit to be set
  readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]);  // Read the six raw data and ST2 registers sequentially into data array
  uint8_t c = rawData[6]; // End data read by reading ST2 register
    if(!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
    destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
    destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  // Data stored as little Endian
    destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ; 
   }
  }
}
//dah bta3 el magnometer feh moshkla fe eladdress lsa hashofo
void initAK8963(float * destination)
{
  // First extract the factory calibration for each magnetometer axis
  uint8_t rawData[3];  // x/y/z gyro calibration data stored here
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer  
  delay(10);
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
  delay(10);
  readBytes(AK8963_ADDRESS, AK8963_ASAX, 3, &rawData[0]);  // Read the x-, y-, and z-axis calibration values
  destination[0] =  (float)(rawData[0] - 128)/256. + 1.;   // Return x-axis sensitivity adjustment values, etc.
  destination[1] =  (float)(rawData[1] - 128)/256. + 1.;  
  destination[2] =  (float)(rawData[2] - 128)/256. + 1.; 
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer  
  delay(10);
  // Configure the magnetometer for continuous read and highest resolution
  // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
  // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
  writeByte(AK8963_ADDRESS, AK8963_CNTL, Mscale << 4 | Mmode); // Set magnetometer data resolution and sample ODR
  delay(10);
}


void initMPU9250()
{  
 // wake up device
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors 
  delay(100); // Wait for all registers to reset 

 // get stable time source
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else
  delay(200); 
  
 // Configure Gyro and Thermometer
 // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively; 
 // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
 // be higher than 1 / 0.0059 = 170 Hz
 // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
 // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
  writeByte(MPU9250_ADDRESS, CONFIG, 0x03);  

 // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; a rate consistent with the filter update rate 
                                    // determined inset in CONFIG above
 
 // Set gyroscope full scale range
 // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
  uint8_t c = readByte(MPU9250_ADDRESS, GYRO_CONFIG); // get current GYRO_CONFIG register value
 // c = c & ~0xE0; // Clear self-test bits [7:5] 
  c = c & ~0x02; // Clear Fchoice bits [1:0] 
  c = c & ~0x18; // Clear AFS bits [4:3]
  c = c | Gscale << 3; // Set full scale range for the gyro
 // c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c ); // Write new GYRO_CONFIG value to register
  
 // Set accelerometer full-scale range configuration
  c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG); // get current ACCEL_CONFIG register value
 // c = c & ~0xE0; // Clear self-test bits [7:5] 
  c = c & ~0x18;  // Clear AFS bits [4:3]
  c = c | Ascale << 3; // Set full scale range for the accelerometer 
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, c); // Write new ACCEL_CONFIG register value

 // Set accelerometer sample rate configuration
 // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
 // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
  c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG2); // get current ACCEL_CONFIG2 register value
  c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])  
  c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, c); // Write new ACCEL_CONFIG2 register value
 // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates, 
 // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

  // Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
  // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips 
  // can join the I2C bus and all can be controlled by the Arduino as master
   writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);    
   writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
   delay(100);
}


// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void calibrateMPU9250(float * dest1, float * dest2)
{  
  uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
  uint16_t ii, packet_count, fifo_count;
  int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
  
 // reset device
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
  delay(100);
   
 // get stable time source; Auto select clock source to be PLL gyroscope reference if ready 
 // else use the internal oscillator, bits 2:0 = 001
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);  
  writeByte(MPU9250_ADDRESS, PWR_MGMT_2, 0x00);
  delay(200);                                    

// Configure device for bias calculation
  writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x00);   // Disable all interrupts
  writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);      // Disable FIFO
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);   // Turn on internal clock source
  writeByte(MPU9250_ADDRESS, I2C_MST_CTRL, 0x00); // Disable I2C master
  writeByte(MPU9250_ADDRESS, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
  writeByte(MPU9250_ADDRESS, USER_CTRL, 0x0C);    // Reset FIFO and DMP
  delay(15);
  
// Configure MPU6050 gyro and accelerometer for bias calculation
  writeByte(MPU9250_ADDRESS, CONFIG, 0x01);      // Set low-pass filter to 188 Hz
  writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity
 
  uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
  uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

    // Configure FIFO to capture accelerometer and gyro data for bias calculation
  writeByte(MPU9250_ADDRESS, USER_CTRL, 0x40);   // Enable FIFO  
  writeByte(MPU9250_ADDRESS, FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
  delay(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

// At end of sample accumulation, turn off FIFO sensor read
  writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
  readBytes(MPU9250_ADDRESS, FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
  fifo_count = ((uint16_t)data[0] << 8) | data[1];
  packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging
  
  for (ii = 0; ii < packet_count; ii++) {
    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    readBytes(MPU9250_ADDRESS, FIFO_R_W, 12, &data[0]); // read data for averaging
    accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
    accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;    
    gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
    gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
    gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;
    
    accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[1] += (int32_t) accel_temp[1];
    accel_bias[2] += (int32_t) accel_temp[2];
    gyro_bias[0]  += (int32_t) gyro_temp[0];
    gyro_bias[1]  += (int32_t) gyro_temp[1];
    gyro_bias[2]  += (int32_t) gyro_temp[2];
            
}
    accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
    accel_bias[1] /= (int32_t) packet_count;
    accel_bias[2] /= (int32_t) packet_count;
    gyro_bias[0]  /= (int32_t) packet_count;
    gyro_bias[1]  /= (int32_t) packet_count;
    gyro_bias[2]  /= (int32_t) packet_count;
    
  if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
  else {accel_bias[2] += (int32_t) accelsensitivity;}
   
// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
  data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
  data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
  data[3] = (-gyro_bias[1]/4)       & 0xFF;
  data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
  data[5] = (-gyro_bias[2]/4)       & 0xFF;
  
// Push gyro biases to hardware registers
  writeByte(MPU9250_ADDRESS, XG_OFFSET_H, data[0]);
  writeByte(MPU9250_ADDRESS, XG_OFFSET_L, data[1]);
  writeByte(MPU9250_ADDRESS, YG_OFFSET_H, data[2]);
  writeByte(MPU9250_ADDRESS, YG_OFFSET_L, data[3]);
  writeByte(MPU9250_ADDRESS, ZG_OFFSET_H, data[4]);
  writeByte(MPU9250_ADDRESS, ZG_OFFSET_L, data[5]);
  
// Output scaled gyro biases for display in the main program
  dest1[0] = (float) gyro_bias[0]/(float) gyrosensitivity;  
  dest1[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
  dest1[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
// the accelerometer biases calculated above must be divided by 8.

  int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
  readBytes(MPU9250_ADDRESS, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
  accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  readBytes(MPU9250_ADDRESS, YA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  readBytes(MPU9250_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  
  uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
  uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis
  
  for(ii = 0; ii < 3; ii++) {
    if((accel_bias_reg[ii] & mask)) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
  }
  
  // Construct total accelerometer bias, including calculated average accelerometer bias from above
  accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
  accel_bias_reg[1] -= (accel_bias[1]/8);
  accel_bias_reg[2] -= (accel_bias[2]/8);
  
  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  data[1] = (accel_bias_reg[0])      & 0xFF;
  data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  data[3] = (accel_bias_reg[1])      & 0xFF;
  data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
  data[5] = (accel_bias_reg[2])      & 0xFF;
  data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers
 
// Apparently this is not working for the acceleration biases in the MPU-9250
// Are we handling the temperature correction bit properly?
// Push accelerometer biases to hardware registers
  writeByte(MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
  writeByte(MPU9250_ADDRESS, XA_OFFSET_L, data[1]);
  writeByte(MPU9250_ADDRESS, YA_OFFSET_H, data[2]);
  writeByte(MPU9250_ADDRESS, YA_OFFSET_L, data[3]);
  writeByte(MPU9250_ADDRESS, ZA_OFFSET_H, data[4]);
  writeByte(MPU9250_ADDRESS, ZA_OFFSET_L, data[5]);

// Output scaled accelerometer biases for display in the main program
   dest2[0] = (float)accel_bias[0]/(float)accelsensitivity; 
   dest2[1] = (float)accel_bias[1]/(float)accelsensitivity;
   dest2[2] = (float)accel_bias[2]/(float)accelsensitivity;
}


// Similar to Madgwick scheme but uses proportional and integral filtering on the error between estimated reference vectors and
 // measured ones. 
 void QuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
        {
            float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
            float norm;
            float hx, hy, bx, bz;
            float vx, vy, vz, wx, wy, wz;
            float ex, ey, ez;
            float pa, pb, pc;

            // Auxiliary variables to avoid repeated arithmetic
            float q1q1 = q1 * q1;
            float q1q2 = q1 * q2;
            float q1q3 = q1 * q3;
            float q1q4 = q1 * q4;
            float q2q2 = q2 * q2;
            float q2q3 = q2 * q3;
            float q2q4 = q2 * q4;
            float q3q3 = q3 * q3;
            float q3q4 = q3 * q4;
            float q4q4 = q4 * q4;   

            // Normalise accelerometer measurement
            norm = sqrtf(ax * ax + ay * ay + az * az);
            if (norm == 0.0f) return; // handle NaN
            norm = 1.0f / norm;        // use reciprocal for division
            ax *= norm;
            ay *= norm;
            az *= norm;

            // Normalise magnetometer measurement
            norm = sqrtf(mx * mx + my * my + mz * mz);
            if (norm == 0.0f) return; // handle NaN
            norm = 1.0f / norm;        // use reciprocal for division
            mx *= norm;
            my *= norm;
            mz *= norm;

            // Reference direction of Earth's magnetic field
            hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
            hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
            bx = sqrtf((hx * hx) + (hy * hy));
            bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

            // Estimated direction of gravity and magnetic field
            vx = 2.0f * (q2q4 - q1q3);
            vy = 2.0f * (q1q2 + q3q4);
            vz = q1q1 - q2q2 - q3q3 + q4q4;
            wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
            wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
            wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);  

            // Error is cross product between estimated direction and measured direction of gravity
            ex = (ay * vz - az * vy) + (my * wz - mz * wy);
            ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
            ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
            if (Ki > 0.0f)
            {
                eInt[0] += ex;      // accumulate integral error
                eInt[1] += ey;
                eInt[2] += ez;
            }
            else
            {
                eInt[0] = 0.0f;     // prevent integral wind up
                eInt[1] = 0.0f;
                eInt[2] = 0.0f;
            }

            // Apply feedback terms
            gx = gx + Kp * ex + Ki * eInt[0];
            gy = gy + Kp * ey + Ki * eInt[1];
            gz = gz + Kp * ez + Ki * eInt[2];

            // Integrate rate of change of quaternion
            pa = q2;
            pb = q3;
            pc = q4;
            q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
            q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * deltat);
            q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * deltat);
            q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * deltat);

            // Normalise quaternion
            norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
            norm = 1.0f / norm;
            q[0] = q1 * norm;
            q[1] = q2 * norm;
            q[2] = q3 * norm;
            q[3] = q4 * norm;
 
        }

  void TM_TIMER_Init(void) {
	TIM_TimeBaseInitTypeDef TIM_BaseStruct;
	
	/* Enable clock for TIM4 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
/*	
	TIM4 is connected to APB1 bus, which has on F407 device 42MHz clock 				
	But, timer has internal PLL, which double this frequency for timer, up to 84MHz 	
	Remember: Not each timer is connected to APB1, there are also timers connected 	
	on APB2, which works at 84MHz by default, and internal PLL increase 				
	this to up to 168MHz 															
	
	Set timer prescaller 
	Timer count frequency is set with 
	
	timer_tick_frequency = Timer_default_frequency / (prescaller_set + 1)		
	
	In our case, we want a max frequency for timer, so we set prescaller to 0 		
	And our timer will have tick frequency		
	
	timer_tick_frequency = 84000000 / (0 + 1) = 84000000 
*/	
	TIM_BaseStruct.TIM_Prescaler = 0;
	/* Count up */
    TIM_BaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
/*
	Set timer period when it have reset
	First you have to know max value for timer
	In our case it is 16bit = 65535
	To get your frequency for PWM, equation is simple
	
	PWM_frequency = timer_tick_frequency / (TIM_Period + 1)
	
	If you know your PWM frequency you want to have timer period set correct
	
	TIM_Period = timer_tick_frequency / PWM_frequency - 1
	
	In our case, for 10Khz PWM_frequency, set Period to
	
	TIM_Period = 84000000 / 10000 - 1 = 8399
	
	If you get TIM_Period larger than max timer value (in our case 65535),
	you have to choose larger prescaler and slow down timer tick frequency
*/
    TIM_BaseStruct.TIM_Period = 8399; /* 10kHz PWM */
    TIM_BaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_BaseStruct.TIM_RepetitionCounter = 0;
	/* Initialize TIM4 */
    TIM_TimeBaseInit(TIM4, &TIM_BaseStruct);
	/* Start count on TIM4 */
    TIM_Cmd(TIM4, ENABLE);
}
  void get_values ()
  {
    I2C_init(); // initialize I2C1 & I2C2  peripheral
            TM_TIMER_Init();
         calibrateMPU9250(gyroBias, accelBias);
         initMPU9250(); 
         initAK8963(magCalibration);
         
       // If intPin goes high, all data registers have new data
    while (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) 
    {  // On interrupt, check if data ready interrupt
    readAccelData(accelCount);  // Read the x/y/z adc values
    getAres();
    
    // Now we'll calculate the accleration value into actual g's
    ax = (float)accelCount[0]*aRes; // - accelBias[0];  // get actual g value, this depends on scale being set
    ay = (float)accelCount[1]*aRes; // - accelBias[1];   
    az = (float)accelCount[2]*aRes; // - accelBias[2];  
   
    readGyroData(gyroCount);  // Read the x/y/z adc values
    getGres();
 
    // Calculate the gyro value into actual degrees per second
    gx = (float)gyroCount[0]*gRes;  // get actual gyro value, this depends on scale being set
    gy = (float)gyroCount[1]*gRes;  
    gz = (float)gyroCount[2]*gRes; 
    
  
    readMagData(magCount);  // Read the x/y/z adc values
    getMres();
    magbias[0] = +470.;  // User environmental x-axis correction in milliGauss, should be automatically calculated
    magbias[1] = +120.;  // User environmental x-axis correction in milliGauss
    magbias[2] = +125.;  // User environmental x-axis correction in milliGauss
    
    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental corrections
    mx = (float)magCount[0]*mRes*magCalibration[0] - magbias[0];  // get actual magnetometer value, this depends on scale being set
    my = (float)magCount[1]*mRes*magCalibration[1] - magbias[1];  
    mz = (float)magCount[2]*mRes*magCalibration[2] - magbias[2]; 
    
    //el mafrod en now =time
    Now = 60000;
    deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
   // lastUpdate = Now;
    //calculation for roll pitch yaw from Quaternion
    QuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, my, mx, mz);
    yaw   =  atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
    pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
    roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
    pitch *= 180.0f / PI;
    yaw   *= 180.0f / PI; 
    roll *= 180.0f / PI;
    }  }

int initial_pitch_flag = 0; 
uint8_t pitch_first_1;
uint8_t pitch_first_2;
float ipitch;
float iroll;
float iyaw;
short concatenate(uint8_t x, uint8_t y) {
    uint8_t pow = 10;
    while(y >= pow)
        pow *= 10;
    return x * pow + y;        
}
float initial_pitch()
{
  
  initial_pitch_flag = 1;
 /* I2C_init();
  I2C_start(I2C1, SLAVE_ADDRESS<<1, I2C_Direction_Receiver);
 int8_t temp;
  for( int i=0;i<18;i++)
  {
   temp = I2C_read_ack(I2C1);
  
  }
     pitch_first_1= I2C_read_ack(I2C1);
     pitch_first_2= I2C_read_ack(I2C1);
  //pitch_first_1 = 11;
  //pitch_first_2 = 20;
         I2C_stop(I2C1);*/
  
/*short concatenated = concatenate (pitch_first_1 ,pitch_first_2);*/
  printf(" %d\n", pitch);
  ipitch=pitch;
  return pitch  ;
  

}
float not_initial_pitch()
{

//short ipitch =concatenate ( pitch_first_1 , pitch_first_2);
//int8_t pitch_1_stabilized  = pitch_first_1;
//int8_t pitch_2_stabilized = pitch_first_2;
//int8_t temp;
 /*for( int i=0;i<18;i++)
  {
   temp = I2C_read_ack(I2C1);
  
  }*/
  
/*int8_t pitch_1_UNstabilized  = I2C_read_ack(I2C1);
int8_t pitch_2_UNstabilized  = I2C_read_ack(I2C1);
        I2C_stop(I2C1);*/
//uint8_t pitch_1_UNstabilized = 11;
//uint8_t pitch_2_UNstabilized  =20;


/*// first part 
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

*/





//short concatenated =concatenate (pitch_1_UNstabilized,pitch_2_UNstabilized);
//  printf("wrong is  %d\n", concatenated);
  //printf("right is  %d\n", ipitch);

if (pitch>ipitch)
{

  pitch = pitch - (pitch - ipitch);
    //  printf("corrected  is  %d\n", concatenated);

}
else if (pitch<ipitch)
{
pitch= pitch +(ipitch-pitch);
     // printf("corrected  is  %d\n", concatenated);

}
else if (pitch==ipitch) 
 // printf(" %d\n", concatenated);
pitch = ipitch;

printf(" %d\n", pitch);
return pitch ; 
}




float stabilize_pitch ()
{
if (initial_pitch_flag==0) return  initial_pitch();
else return not_initial_pitch();



}

//////////////////////////////////////////////////////////
int initial_yaw_flag = 0; 
//uint8_t yaw_first_1;
//uint8_t yaw_first_2;

float initial_yaw()
{
  
  initial_yaw_flag = 1;
 /*I2C_init();
  I2C_start(I2C1, SLAVE_ADDRESS<<1, I2C_Direction_Receiver);
  uint8_t temp;
  for( int i=0;i<18;i++)
  {
   temp = I2C_read_ack(I2C1);
  
  }
   yaw_first_1= I2C_read_ack(I2C1);
     yaw_first_2= I2C_read_ack(I2C1);
 // yaw_first_1 = 11;
  //yaw_first_2 = 20;
       I2C_stop(I2C1);
  */
//short concatenated = concatenate (yaw_first_1 ,yaw_first_2);
  printf(" %d\n", iyaw);
  return iyaw  ;
  

}
float not_initial_yaw()
{

//short iyaw =concatenate ( yaw_first_1 , yaw_first_2);
//uint8_t yaw_1_stabilized  = yaw_first_1;
//uint8_t yaw_2_stabilized = yaw_first_2;
//uint8_t temp;
 /*for( int i=0;i<22;i++)
  {
   temp = I2C_read_ack(I2C1);
  
  }
  
uint8_t yaw_1_UNstabilized  = I2C_read_ack(I2C1);
uint8_t yaw_2_UNstabilized  = I2C_read_ack(I2C1);
          I2C_stop(I2C1);
//uint8_t yaw_1_UNstabilized = 13;
//uint8_t yaw_2_UNstabilized  =20;

*/
/*// first part 
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
yaw_2_stabilized= yaw_2_UNstabilized +(yaw_2_stabilized-pitch_2_UNstabilized);
}
else if (yaw_2_UNstabilized==yaw_2_stabilized) ;

*/





/*short concatenated =concatenate (yaw_1_UNstabilized,yaw_2_UNstabilized);
  printf("wrong is  %d\n", concatenated);
  printf("right is  %d\n", iyaw);*/

if (yaw>iyaw)
{

  yaw = yaw - (yaw - iyaw);
      //printf("corrected  is  %d\n", concatenated);

}
else if (yaw<iyaw)
{
yaw= yaw +(iyaw-yaw);
    //  printf("corrected  is  %d\n", concatenated);

}
else if (yaw==iyaw) ;
 // printf(" %d\n", concatenated);
yaw = iyaw;

  printf(" %d\n", yaw);

return yaw ; 
}




float stabilize_yaw ()
{
if (initial_yaw_flag==0) return  initial_yaw();
else return not_initial_yaw();



}

//////////////////////////////////////////////////////////


float to_motor_angles (float imu_angle)
{
  return    (31.0/6)*imu_angle+1470;
}

int main(void){

printf("hello");

  
      float to_motor_pitch,to_motor_yaw;  
	
        
    while (1)
    {
              get_values();

    stabilize_pitch();
    to_motor_pitch=to_motor_angles(pitch);
    stabilize_yaw();
    to_motor_yaw=to_motor_angles(yaw);
    
    
    }
 return 0;
  
  
}

