#include <stdio.h>
#include <math.h>
// MPU-9150 Accelerometer + Gyro + Compass + Temperature
// -----------------------------
//
// By arduino.cc user "frtrobotik" (Tobias Hübner)
//
//
// July 2013
//      first version
//
// Open Source / Public Domain
//
// Using Arduino 1.0.1
// It will not work with an older version,
// since Wire.endTransmission() uses a parameter
// to hold or release the I2C bus.
//
// Documentation:
// - The InvenSense documents:
//   - "MPU-9150 Product Specification Revision 4.0",
//     PS-MPU-9150A.pdf
//   - "MPU-9150 Register Map and Descriptions Revision 4.0",
//     RM-MPU-9150A-00.pdf
//   - "MPU-9150 9-Axis Evaluation Board User Guide"
//     AN-MPU-9150EVB-00.pdf
//
// The accuracy is 16-bits.
//
// Some parts are copied by the MPU-6050 Playground page.
// playground.arduino.cc/Main/MPU-6050
// There are more Registervalues. Here are only the most
// nessecary ones to get started with this sensor.

#include <Wire.h>

// Register names according to the datasheet.
// According to the InvenSense document
// "MPU-9150 Register Map and Descriptions Revision 4.0",

#define MPU9150_SELF_TEST_X        0x0D   // R/W
#define MPU9150_SELF_TEST_Y        0x0E   // R/W
#define MPU9150_SELF_TEST_Z        0x0F   // R/W
#define MPU9150_SELF_TEST_A        0x10   // R/W
#define MPU9150_SMPLRT_DIV         0x19   // R/W
#define MPU9150_CONFIG             0x1A   // R/W
#define MPU9150_GYRO_CONFIG        0x1B   // R/W
#define MPU9150_ACCEL_CONFIG       0x1C   // R/W
#define MPU9150_FF_THR             0x1D   // R/W
#define MPU9150_FF_DUR             0x1E   // R/W
#define MPU9150_MOT_THR            0x1F   // R/W
#define MPU9150_MOT_DUR            0x20   // R/W
#define MPU9150_ZRMOT_THR          0x21   // R/W
#define MPU9150_ZRMOT_DUR          0x22   // R/W
#define MPU9150_FIFO_EN            0x23   // R/W
#define MPU9150_I2C_MST_CTRL       0x24   // R/W
#define MPU9150_I2C_SLV0_ADDR      0x25   // R/W
#define MPU9150_I2C_SLV0_REG       0x26   // R/W
#define MPU9150_I2C_SLV0_CTRL      0x27   // R/W
#define MPU9150_I2C_SLV1_ADDR      0x28   // R/W
#define MPU9150_I2C_SLV1_REG       0x29   // R/W
#define MPU9150_I2C_SLV1_CTRL      0x2A   // R/W
#define MPU9150_I2C_SLV2_ADDR      0x2B   // R/W
#define MPU9150_I2C_SLV2_REG       0x2C   // R/W
#define MPU9150_I2C_SLV2_CTRL      0x2D   // R/W
#define MPU9150_I2C_SLV3_ADDR      0x2E   // R/W
#define MPU9150_I2C_SLV3_REG       0x2F   // R/W
#define MPU9150_I2C_SLV3_CTRL      0x30   // R/W
#define MPU9150_I2C_SLV4_ADDR      0x31   // R/W
#define MPU9150_I2C_SLV4_REG       0x32   // R/W
#define MPU9150_I2C_SLV4_DO        0x33   // R/W
#define MPU9150_I2C_SLV4_CTRL      0x34   // R/W
#define MPU9150_I2C_SLV4_DI        0x35   // R  
#define MPU9150_I2C_MST_STATUS     0x36   // R
#define MPU9150_INT_PIN_CFG        0x37   // R/W
#define MPU9150_INT_ENABLE         0x38   // R/W
#define MPU9150_INT_STATUS         0x3A   // R  
#define MPU9150_ACCEL_XOUT_H       0x3B   // R  
#define MPU9150_ACCEL_XOUT_L       0x3C   // R  
#define MPU9150_ACCEL_YOUT_H       0x3D   // R  
#define MPU9150_ACCEL_YOUT_L       0x3E   // R  
#define MPU9150_ACCEL_ZOUT_H       0x3F   // R  
#define MPU9150_ACCEL_ZOUT_L       0x40   // R  
#define MPU9150_TEMP_OUT_H         0x41   // R  
#define MPU9150_TEMP_OUT_L         0x42   // R  
#define MPU9150_GYRO_XOUT_H        0x43   // R  
#define MPU9150_GYRO_XOUT_L        0x44   // R  
#define MPU9150_GYRO_YOUT_H        0x45   // R  
#define MPU9150_GYRO_YOUT_L        0x46   // R  
#define MPU9150_GYRO_ZOUT_H        0x47   // R  
#define MPU9150_GYRO_ZOUT_L        0x48   // R  
#define MPU9150_EXT_SENS_DATA_00   0x49   // R  
#define MPU9150_EXT_SENS_DATA_01   0x4A   // R  
#define MPU9150_EXT_SENS_DATA_02   0x4B   // R  
#define MPU9150_EXT_SENS_DATA_03   0x4C   // R  
#define MPU9150_EXT_SENS_DATA_04   0x4D   // R  
#define MPU9150_EXT_SENS_DATA_05   0x4E   // R  
#define MPU9150_EXT_SENS_DATA_06   0x4F   // R  
#define MPU9150_EXT_SENS_DATA_07   0x50   // R  
#define MPU9150_EXT_SENS_DATA_08   0x51   // R  
#define MPU9150_EXT_SENS_DATA_09   0x52   // R  
#define MPU9150_EXT_SENS_DATA_10   0x53   // R  
#define MPU9150_EXT_SENS_DATA_11   0x54   // R  
#define MPU9150_EXT_SENS_DATA_12   0x55   // R  
#define MPU9150_EXT_SENS_DATA_13   0x56   // R  
#define MPU9150_EXT_SENS_DATA_14   0x57   // R  
#define MPU9150_EXT_SENS_DATA_15   0x58   // R  
#define MPU9150_EXT_SENS_DATA_16   0x59   // R  
#define MPU9150_EXT_SENS_DATA_17   0x5A   // R  
#define MPU9150_EXT_SENS_DATA_18   0x5B   // R  
#define MPU9150_EXT_SENS_DATA_19   0x5C   // R  
#define MPU9150_EXT_SENS_DATA_20   0x5D   // R  
#define MPU9150_EXT_SENS_DATA_21   0x5E   // R  
#define MPU9150_EXT_SENS_DATA_22   0x5F   // R  
#define MPU9150_EXT_SENS_DATA_23   0x60   // R  
#define MPU9150_MOT_DETECT_STATUS  0x61   // R  
#define MPU9150_I2C_SLV0_DO        0x63   // R/W
#define MPU9150_I2C_SLV1_DO        0x64   // R/W
#define MPU9150_I2C_SLV2_DO        0x65   // R/W
#define MPU9150_I2C_SLV3_DO        0x66   // R/W
#define MPU9150_I2C_MST_DELAY_CTRL 0x67   // R/W
#define MPU9150_SIGNAL_PATH_RESET  0x68   // R/W
#define MPU9150_MOT_DETECT_CTRL    0x69   // R/W
#define MPU9150_USER_CTRL          0x6A   // R/W
#define MPU9150_PWR_MGMT_1         0x6B   // R/W
#define MPU9150_PWR_MGMT_2         0x6C   // R/W
#define MPU9150_FIFO_COUNTH        0x72   // R/W
#define MPU9150_FIFO_COUNTL        0x73   // R/W
#define MPU9150_FIFO_R_W           0x74   // R/W
#define MPU9150_WHO_AM_I           0x75   // R

//MPU9150 Compass
#define MPU9150_CMPS_XOUT_L        0x4A   // R
#define MPU9150_CMPS_XOUT_H        0x4B   // R
#define MPU9150_CMPS_YOUT_L        0x4C   // R
#define MPU9150_CMPS_YOUT_H        0x4D   // R
#define MPU9150_CMPS_ZOUT_L        0x4E   // R
#define MPU9150_CMPS_ZOUT_H        0x4F   // R


// I2C address 0x69 could be 0x68 depends on your wiring. 

// I2C SLAVE ADDRESS of the MPU-9150 is given by
// if A0=1 (HIGH) : 0x69
// if A0=0 (LOW) : 0x68
// Currently, the my MPU-9150 has a short between the A0, FSYNC, and CIN
// FSYNC needs to be LOGIC_LOW to disable the FSYNC functionality

#define ADDRESS_OF_IMU (0x68)
int MPU9150_I2C_ADDRESS = ADDRESS_OF_IMU;


//Variables where our values can be stored
int cmps[3];
int accl[3];
int gyro[3];
int temp;


//******* STATE.c




#define PART_TM4C123GH6PM

// Defines standard sized integers (independent of Architecture) like uint32_t and uint8_t;
#include <stdint.h>
// Defines a BOOL datatype using TYPECAST of standard integer
#include <stdbool.h>

// I2C|TWI Arduino(Energia) Function Library
#include <Wire.h>

// Serial Peripheral Interface (SPI) Arduino(Energia) Function Library
#include <SPI.h>

// Energia Library 
#include <Energia.h>



//	Defines hardware specific MACROS for TM4C123GH6PM (Tiva C 123)
#include "inc/tm4c123gh6pm.h"
//  TivaWare Driver Library GPIO handling
#include "driverlib/gpio.h"
//  TivaWare Pin Map, needs MACRO PART_TM4C123GH6PM defined in "inc/tm4c123gh6pm"
#include "driverlib/pin_map.h"
//	TivaWare System Control Library
//	Uses to SET/GET System Clock Speed
#include "driverlib/sysctl.h"
//	TivaWare Driver Library I2C|TWI 
#include "driverlib/i2c.h"

// Includes for Tiva WORKSHOP for TIMER INTERRUPTS

#include <stdint.h>
#include <stdbool.h>
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"

// TM4C123GH6PM has a MAXIMUM SYSTEM CLOCK RATE of 80 [MHz] ( when running from the INTERNAL OSCILLATORS/PHASE-LOCKED LOOP

// Define MACRO for the CLOCK_RATE in [Hz] 
#define CLOCK_RATE 80000000

// Define RED,BLUE,GREEN pin(Arduino|Energia) values
#define RED 30
#define BLUE 40
#define GREEN 39

// AMOUNT OF TIME BETWEEN STATE TRANSITIONS IS given by A/B = 1/4 = 0.25[s]
// (A,B) are INTEGERS, such that A>0 & B>0
#define A 1
#define B 20

// Define the BAUDRATE of UART0(Serial) to 9216000
#define BAUDRATE 115200

//  Declare a GLOBAL 8-bit STATE VARIABLE
uint8_t state_led;

//  Declare PROTOTYPES for MPU9150 REGISTER READ/WRITE Functions
int MPU9150_writeSensor(int addr,int data);
int MPU9150_readSensor(int addr);
int MPU9150_readSensor(int addrL, int addrH);


// TIMER0 INTERRUPT SERVICE ROUTINE (ISR)
void Timer0_isr(void);

//	Declare PROTOTYPE for my own DELAY FUNCTION
void mydelay( uint32_t milliseconds );


//  void setup(void) runs ONCE when the program just STARTS
void setup()
{
  // First SET the SYSTEM CLOCK to 80 [MHz]
  //  Sets the Clock DIVIDER to 2.5, so that SYS_CLOCK runs at 200/2.5 ==> 80 [MHz]
  SysCtlClockSet( SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

  uint32_t Timer0_Period;
  // Enable & Configure Serial(UART0) to BAUDRATE = 9216000
  Serial.begin(BAUDRATE);  

  // Configure PINS for LED OUTPUT
  pinMode(RED,OUTPUT);
  pinMode(BLUE,OUTPUT);
  pinMode(GREEN,OUTPUT);

  // Set LED OUTPUT pins to OFF initially
  digitalWrite(RED,LOW);
  digitalWrite(BLUE,LOW);
  digitalWrite(GREEN,LOW);
  state_led=0;  // initialize the STATE to STATE0

    // Initialize the 'Wire' class for the I2C-bus.
  Wire.begin();

  // Clear the 'sleep' bit to start the sensor.
  MPU9150_writeSensor(MPU9150_PWR_MGMT_1, 0);
  int temp0;
  temp0=  MPU9150_readSensor(0x1c);
  temp0 |= (0x10);
  temp0 &=~(0x08);

  
  MPU9150_writeSensor(0x1c,temp0);
  
  MPU9150_writeSensor(0x19,0x0f);
  // disable mpu9150 i2c master mode
  //  I2C_MST_EN (0x6a[5]=0)
  MPU9150_writeSensor(0x6a,MPU9150_readSensor(0x6a) & (~(0x20)) ); 
  // enable i2c pass-through, mpu9150 i2c mastermode must be disabled, that is I2C_MST_EN (0x6a[5]=0)
  // I2C_BYPASS_EN (0x37[1]=1)
  MPU9150_writeSensor(0x37,MPU9150_readSensor(0x37) | 0x02);
  MPU9150_setupCompass();

  
  SysCtlPeripheralEnable( SYSCTL_PERIPH_TIMER0); // Enable Timer0

    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC); // Set Timer0 mode PERIODIC



    // Timer0_Period ==> 80e6 * A / B, (1/2)=0.5
  Timer0_Period = ( 80000000 * A ) / B;      
  TimerLoadSet( TIMER0_BASE, TIMER_A, Timer0_Period-1);


  // REGISTER ISR to TIMER0 INTERRUPT
  TimerIntRegister( TIMER0_BASE, TIMER_A, Timer0_isr );

  // Enable Interrupts from Timer0_A
  IntEnable( INT_TIMER0A);


  // Set Timer Interrupt Condition and Enable Timer Interrupt
  TimerIntEnable( TIMER0_BASE, TIMER_TIMA_TIMEOUT);

  // Enable INTERRUPTS for the SYSTEM
  IntMasterEnable();

  // Start Timer0	
  TimerEnable(TIMER0_BASE, TIMER_A);        
}


// void loop(void) function that keeps REPEATED CALLED in INFINITE LOOP after setup(void) EXITS 

void loop()
{
  // SPIN in INFINITE FOR LOOP, waiting for Timer0 Interrupts
  for(;;)
  {
  }

}



void mydelay( uint32_t ms )
{
  //uuint64_t ticks;
  delay( ms );
}
char buf1[256];
float cx,cy,cz;
float ccf = 0.29269;
void Timer0_isr(void)
{
  // Clear TIMER1 INTERRUPT
  TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);


  /*
	
   	// Send a indicator that ISR has been CALLED through UART0[Serial]
   	
   Serial.print("Timer0_isr() called");
   Serial.println("");
   */


  // Output the GYRO data and ACCEL data (with spaces in between each sample)
  // set of 3*2=6 sample values at output on the same line;
  // this is done through the Serial (UART) peripheral [Default]
/*
  Serial.print(MPU9150_readSensor(MPU9150_CMPS_XOUT_L,MPU9150_CMPS_XOUT_H));
  Serial.print(" ");
  Serial.print(MPU9150_readSensor(MPU9150_CMPS_YOUT_L,MPU9150_CMPS_YOUT_H));
  Serial.print(" ");
  Serial.print(MPU9150_readSensor(MPU9150_CMPS_ZOUT_L,MPU9150_CMPS_ZOUT_H));
  Serial.print(" ");
  Serial.print(MPU9150_readSensor(MPU9150_GYRO_XOUT_L,MPU9150_GYRO_XOUT_H));
  Serial.print(" ");
  Serial.print(MPU9150_readSensor(MPU9150_GYRO_YOUT_L,MPU9150_GYRO_YOUT_H));
  Serial.print(" ");
  Serial.print(MPU9150_readSensor(MPU9150_GYRO_ZOUT_L,MPU9150_GYRO_ZOUT_H));
  Serial.print(" ");
 
  Serial.print( MPU9150_readSensor( MPU9150_ACCEL_XOUT_L,MPU9150_ACCEL_XOUT_H) );
  Serial.print(" ");
  Serial.print( MPU9150_readSensor( MPU9150_ACCEL_YOUT_L,MPU9150_ACCEL_YOUT_H) );
  Serial.print(" ");
  Serial.print( MPU9150_readSensor( MPU9150_ACCEL_ZOUT_L,MPU9150_ACCEL_ZOUT_H) ); */
  cx=MPU9150_readSensor(MPU9150_CMPS_XOUT_L,MPU9150_CMPS_XOUT_H)*ccf;
  cy=MPU9150_readSensor(MPU9150_CMPS_YOUT_L,MPU9150_CMPS_YOUT_H)*ccf;
  cz=MPU9150_readSensor(MPU9150_CMPS_ZOUT_L,MPU9150_CMPS_ZOUT_H)*ccf;
  sprintf(buf1,"%f, %f, %f %f",cx,cy,cz,sqrt(cx*cx+cy*cy+cz*cz));
  Serial.println(buf1);
  

} 



//http://pansenti.wordpress.com/2013/03/26/pansentis-invensense-mpu-9150-software-for-arduino-is-now-on-github/
//Thank you to pansenti for setup code.
//I will documented this one later.
void MPU9150_setupCompass(){
  MPU9150_I2C_ADDRESS = 0x0C;      //change Address to Compass

  MPU9150_writeSensor(0x0A, 0x00); //PowerDownMode
  MPU9150_writeSensor(0x0A, 0x0F); //SelfTest
  MPU9150_writeSensor(0x0A, 0x00); //PowerDownMode

  //MPU9150_I2C_ADDRESS = 0x68;      //change Adress to MPU
  
  MPU9150_I2C_ADDRESS = ADDRESS_OF_IMU;      //change Adress to MPU  

  MPU9150_writeSensor(0x24, 0x40); //Wait for Data at Slave0
  MPU9150_writeSensor(0x25, 0x8C); //Set i2c address at slave0 at 0x0C
  MPU9150_writeSensor(0x26, 0x02); //Set where reading at slave 0 starts
  MPU9150_writeSensor(0x27, 0x88); //set offset at start reading and enable
  MPU9150_writeSensor(0x28, 0x0C); //set i2c address at slv1 at 0x0C
  MPU9150_writeSensor(0x29, 0x0A); //Set where reading at slave 1 starts
  MPU9150_writeSensor(0x2A, 0x81); //Enable at set length to 1
  MPU9150_writeSensor(0x64, 0x01); //overvride register
  MPU9150_writeSensor(0x67, 0x03); //set delay rate
  MPU9150_writeSensor(0x01, 0x80);

  MPU9150_writeSensor(0x64, 0x04); //set i2c slv4 delay
  MPU9150_writeSensor(0x64, 0x00); //override register
  MPU9150_writeSensor(0x6A, 0x00); //clear usr setting
  MPU9150_writeSensor(0x64, 0x01); //override register
  MPU9150_writeSensor(0x6A, 0x20); //enable master i2c mode
  MPU9150_writeSensor(0x34, 0x13); //disable slv4
}

////////////////////////////////////////////////////////////
///////// I2C functions to get easier all values ///////////
////////////////////////////////////////////////////////////

int MPU9150_readSensor(int addrL, int addrH){
  Wire.beginTransmission(MPU9150_I2C_ADDRESS);
  Wire.write(addrL);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU9150_I2C_ADDRESS, 1, true);
  byte L = Wire.read();

  Wire.beginTransmission(MPU9150_I2C_ADDRESS);
  Wire.write(addrH);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU9150_I2C_ADDRESS, 1, true);
  byte H = Wire.read();

  return (int16_t)((H<<8)+L);
}

int MPU9150_readSensor(int addr){
  Wire.beginTransmission(MPU9150_I2C_ADDRESS);
  Wire.write(addr);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU9150_I2C_ADDRESS, 1, true);
  return Wire.read();
}

int MPU9150_writeSensor(int addr,int data){
  Wire.beginTransmission(MPU9150_I2C_ADDRESS);
  Wire.write(addr);
  Wire.write(data);
  Wire.endTransmission(true);

  return 1;
}

