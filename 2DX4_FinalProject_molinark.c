/*
Kyle Molinari
COMPENG 2DX4 Final Project
400136596
molinark
*/

#include <stdint.h>
#include "tm4c1294ncpdt.h"
#include "vl53l1x_api.h"
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"

// The VL53L1X uses a slightly different way to define the default address of 0x29
// The I2C protocol defintion states that a 7-bit address is used for the device
// The 7-bit address is stored in bit 7:1 of the address register.  Bit 0 is a binary
// value that indicates if a write or read is to occur.  The manufacturer lists the 
// default address as 0x52 (0101 0010).  This is 0x29 (010 1001) with the read/write bit
// alread set to 0.
//uint16_t	dev = 0x29;
uint16_t	dev=0x52;

int status=0;

volatile int IntCount;

//device in interrupt mode (GPIO1 pin signal)
#define isInterrupt 1 /* If isInterrupt = 1 then device working in interrupt mode, else device working in polling mode */

void I2C_Init(void);
void UART_Init(void);
void PortG_Init(void);
void VL53L1X_XSHUT(void);


//direction: 0 for clockwise, 1 for counterclockwise
//revs: any number of revolutions
//speed: any number from 0-1, as a percentage of "max" speed


void stepper_motor(int direction, float revs, float speed, uint8_t RangeStatus, uint8_t dataReady, uint16_t Distance, uint16_t debugArray[100], int sampleAngle){
	int count = 0; //initialize count to keep track of how long we need to keep the motor on
	int cycles = 0; //initialize cycles to keep track of the angle in order to flash onboard led
	int i=0;
	float lastAngle = 0; //dummy value which will be overwritten later
	float currentAngle;
	GPIO_PORTL_DATA_R = 0b0; // turn off offboard led
	//clockwise turn
	if(direction == 0){
		for(count=0; count<512*revs; count++){
			//if stop button is pressed turn everything off and exit the function
			if((GPIO_PORTK_DATA_R&0b10)==0b10){
				GPIO_PORTM_DATA_R = 0b0000; //turn off stepper motor
				GPIO_PORTL_DATA_R = 0b100000; //turn on offboard led
				GPIO_PORTN_DATA_R = 0b0; // turn off onboard led
				UART_printf("restart\n");
				return;
			}
			GPIO_PORTM_DATA_R = 0b1100;
			SysTick_Wait10ms(2/speed);
			GPIO_PORTM_DATA_R = 0b0110;
			SysTick_Wait10ms(2/speed);
			GPIO_PORTM_DATA_R = 0b0011;
			SysTick_Wait10ms(2/speed);
			GPIO_PORTM_DATA_R = 0b1001;
			SysTick_Wait10ms(2/speed);
			cycles++;
			GPIO_PORTN_DATA_R = 0b0; //turn off angle led
			
			currentAngle = (int)(360*cycles/512); // find current angle
			
			/*
			if statement checks that the current angle is a multiple of 10 degrees and that the current angle is different from the last angle
			this prevents multiple distance measurements from being printed for the same angle repetitively
			this ensures only 1 distance measurement is printed at each 10 degree increment
			*/
			if(((int)((360*cycles/512)%sampleAngle)==0) && lastAngle != currentAngle){
				//GPIO_PORTN_DATA_R = 0b10; //turn on led
				
				lastAngle = (int)(360*cycles/512); //set last angle
				
				// insert sensor collecting data interrupt here
				while (dataReady == 0){
					status = VL53L1X_CheckForDataReady(dev, &dataReady);
					VL53L1_WaitMs(dev, 0);
			}

				dataReady = 0;
				status = VL53L1X_GetRangeStatus(dev, &RangeStatus);
				status = VL53L1X_GetDistance(dev, &Distance);
				
				debugArray[i] = Distance;
				i++;

				status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
				
				sprintf(printf_buffer,"%u\r\n", Distance);
				UART_printf(printf_buffer);
				}
			
		}
		GPIO_PORTM_DATA_R = 0b0000; //turn off stepper motor
		GPIO_PORTL_DATA_R = 0b100000; //turn on offboard led
		GPIO_PORTN_DATA_R = 0b0; // turn off onboard led
		return;
	}
	//counterclockwise turn
	if(direction == 1){
		for(count=0; count<512*revs; count++){
			//if stop button is pressed turn everything off and exit the function
			if((GPIO_PORTK_DATA_R&0b10)==0b10){
				GPIO_PORTM_DATA_R = 0b0000; //turn off stepper motor
				GPIO_PORTL_DATA_R = 0b100000; //turn on offboard led
				GPIO_PORTN_DATA_R = 0b0; // turn off onboard led
				UART_printf("restart\n");
				return;
			}
			GPIO_PORTM_DATA_R = 0b1001;
			SysTick_Wait10ms(2/speed);
			GPIO_PORTM_DATA_R = 0b0011;
			SysTick_Wait10ms(2/speed);
			GPIO_PORTM_DATA_R = 0b0110;
			SysTick_Wait10ms(2/speed);
			GPIO_PORTM_DATA_R = 0b1100;
			SysTick_Wait10ms(2/speed);
			cycles++;
			GPIO_PORTN_DATA_R = 0b0; //turn off angle led
			
			currentAngle = (int)(360*cycles/512); // find current angle

			/*
			if statement checks that the current angle is a multiple of ____ degrees and that the current angle is different from the last angle
			this prevents multiple distance measurements from being printed for the same angle repetitively
			this ensures only 1 distance measurement is printed at each ____ degree increment
			*/
			if(((int)((360*cycles/512)%sampleAngle)==0) && lastAngle != currentAngle){
					
				lastAngle = (int)(360*cycles/512); //set last angle
				
				while (dataReady == 0){
					status = VL53L1X_CheckForDataReady(dev, &dataReady);
					VL53L1_WaitMs(dev, 0); 
			}

				dataReady = 0;
				status = VL53L1X_GetRangeStatus(dev, &RangeStatus);
				status = VL53L1X_GetDistance(dev, &Distance);
				
				debugArray[i] = Distance;
				i++;

				status = VL53L1X_ClearInterrupt(dev); // clear interrupt has to be called to enable next interrupt
		
				sprintf(printf_buffer,"%u\r\n", Distance);
				UART_printf(printf_buffer);
				
				}
			
		}
		GPIO_PORTM_DATA_R = 0b0000; //turn off stepper motor
		GPIO_PORTL_DATA_R = 0b100000; //turn on offboard led
		GPIO_PORTN_DATA_R = 0b0; // turn off onboard led
		return;
	}
}

//output pins PM0-3 for stepper motor
void CONFIG_PORTM(){
	SYSCTL_RCGCGPIO_R |=SYSCTL_RCGCGPIO_R11;
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R11) == 0){};
	GPIO_PORTM_DIR_R |= 0b1111; //output
	GPIO_PORTM_DEN_R |= 0b1111; //enable pins 0-3
}

//onboard LED output PN1 - for 45 degree angle flash
void CONFIG_PORTN(){
	SYSCTL_RCGCGPIO_R |=SYSCTL_RCGCGPIO_R12;
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R12) == 0){};
	GPIO_PORTN_DIR_R |= 0b10; //pin 1 as output
	GPIO_PORTN_DEN_R |= 0b10; //enable pin 1
	GPIO_PORTN_DATA_R = 0b00; //turn led off for default
}

//offboard LED output PL5 - on when motor is off
void CONFIG_PORTL(){
	SYSCTL_RCGCGPIO_R |=SYSCTL_RCGCGPIO_R10;
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R10) == 0){};
	GPIO_PORTL_DIR_R |= 0b100000; //set pin 5 as output
	GPIO_PORTL_DEN_R |= 0b100000; //enable pin 5
	GPIO_PORTL_DATA_R = 0b100000; //turn led on for default
}

//input pins PK0 (start) and PK1 (immediate stop)
void CONFIG_PORTK(){
	SYSCTL_RCGCGPIO_R |=SYSCTL_RCGCGPIO_R9;
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R9) == 0){};
	GPIO_PORTK_DIR_R |= 0b00; //set pins 0 and 1 as input
	GPIO_PORTK_DEN_R |= 0b11; //enable pins 0 and 1
}

//capture values from VL53L1X for inspection
uint16_t debugArray[100];

int main(void) {
  uint8_t byteData, sensorState=0, myByteArray[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} , i=0;
  uint16_t wordData;
  uint8_t ToFSensor = 1; // 0=Left, 1=Center(default), 2=Right
  uint16_t Distance;
  uint16_t SignalRate;
  uint16_t AmbientRate;
  uint16_t SpadNum; 
  uint8_t RangeStatus;
  uint8_t dataReady;

	//initialize
	CONFIG_PORTM();
	CONFIG_PORTN();
	CONFIG_PORTK();
	CONFIG_PORTL();
	PLL_Init();	
	SysTick_Init();
	onboardLEDs_Init();
	I2C_Init();
	UART_Init();
	

/* Those basic I2C read functions can be used to check your own I2C functions */
  status = VL53L1_RdByte(dev, 0x010F, &byteData);					// This is the model ID.  Expected returned value is 0xEA
  myByteArray[i++] = byteData;

  status = VL53L1_RdByte(dev, 0x0110, &byteData);					// This is the module type.  Expected returned value is 0xCC
  myByteArray[i++] = byteData;
	
	status = VL53L1_RdWord(dev, 0x010F, &wordData);
	status = VL53L1X_GetSensorId(dev, &wordData);

	//sprintf(printf_buffer,"Model_ID=0x%x , Module_Type=0x%x\r\n",myByteArray[0],myByteArray[1]);
	//UART_printf(printf_buffer);

	// Booting ToF chip
	while(sensorState==0){
		status = VL53L1X_BootState(dev, &sensorState);
		SysTick_Wait10ms(10);
  }
	
	//UART_printf("ToF Chip Booted!\r\n");
 	//UART_printf("One moment...\r\n");
	
	status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/

	
  /* This function must to be called to initialize the sensor with the default setting  */
  status = VL53L1X_SensorInit(dev);
	//Status_Check("SensorInit", status);

	
  /* Optional functions to be used to change the main ranging parameters according the application requirements to get the best ranging performances */
  status = VL53L1X_SetDistanceMode(dev, 2); /* 1=short, 2=long */
  status = VL53L1X_StartRanging(dev);   /* This function has to be called to enable the ranging */
	
	FlashAllLEDs();
	int stepAngle = 3; //angle between each distance measurement. recommended range is 1-45.
	int totalRotations = 1; 

	while(1){
		//delay until start button is pressed
		while((GPIO_PORTK_DATA_R&0b1)==0b0){
			SysTick_Wait10ms(1);
		}
		
		sprintf(printf_buffer,"%u,\n", stepAngle);
		UART_printf(printf_buffer);
		
		//direction, revs, speed	
		stepper_motor(1,totalRotations,1, RangeStatus, dataReady, Distance, debugArray, stepAngle);
		
	}
	
  VL53L1X_StopRanging(dev);

  while(1) {}

}


#define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  // Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  // Acknowledge Address
#define I2C_MCS_STOP            0x00000004  // Generate STOP
#define I2C_MCS_START           0x00000002  // Generate START
#define I2C_MCS_ERROR           0x00000002  // Error
#define I2C_MCS_RUN             0x00000001  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  // I2C Busy
#define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable

#define MAXRETRIES              5           // number of receive attempts before giving up
void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;           // activate I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;          // activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};// ready?

    GPIO_PORTB_AFSEL_R |= 0x0C;           // 3) enable alt funct on PB2,3       0b00001100
    GPIO_PORTB_ODR_R |= 0x08;             // 4) enable open drain on PB3 only

    GPIO_PORTB_DEN_R |= 0x0C;             // 5) enable digital I/O on PB2,3
//    GPIO_PORTB_AMSEL_R &= ~0x0C;          // 7) disable analog functionality on PB2,3

                                                                                // 6) configure PB2,3 as I2C
//  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00003300;
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    //TED
    I2C0_MCR_R = I2C_MCR_MFE;                      // 9) master function enable
    I2C0_MTPR_R = 0b0000000000000101000000000111011;                                        // 8) configure for 100 kbps clock (added 8 clocks of glitch suppression ~50ns)
//    I2C0_MTPR_R = 0x3B;                                        // 8) configure for 100 kbps clock
        
  // 20*(TPR+1)*20ns = 10us, with TPR=24
    // TED 100 KHz
    //     CLK_PRD = 8.3ns
    //    TIMER_PRD = 1
    //    SCL_LP = 6
    //    SCL_HP = 4
    //    10us = 2 * (1 + TIMER_PRD) * (SCL_LP + SCL_HP) * CLK_PRD
    //    10us = 2 * (1+TIMER+PRD) * 10 * 8.3ns
    //  TIMER_PRD = 59 (0x3B)
    //
    // TIMER_PRD is a 6-bit value.  This 0-127
    //    @0: 2 * (1+ 0) * 10 * 8.3ns --> .1667us or 6.0MHz
    //  @127: 2 * (1+ 127) * 10 * 8.3ns --> 47kHz
    
    
}

//The VL53L1X needs to be reset using XSHUT.  We will use PG0
void PortG_Init(void){
    //Use PortG0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;                // activate clock for Port N
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};    // allow time for clock to stabilize
    GPIO_PORTG_DIR_R &= 0x00;                                        // make PG0 in (HiZ)
  GPIO_PORTG_AFSEL_R &= ~0x01;                                     // disable alt funct on PG0
  GPIO_PORTG_DEN_R |= 0x01;                                        // enable digital I/O on PG0
                                                                                                    // configure PG0 as GPIO
  //GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFFFFF00)+0x00000000;
  GPIO_PORTG_AMSEL_R &= ~0x01;                                     // disable analog functionality on PN0

    return;
}

//XSHUT     This pin is an active-low shutdown input; the board pulls it up to VDD to enable the sensor by default. Driving this pin low puts the sensor into hardware standby. This input is not level-shifted.
void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01;                                        // make PG0 out
    GPIO_PORTG_DATA_R &= 0b11111110;                                 //PG0 = 0
    
    SysTick_Wait10ms(10);
    GPIO_PORTG_DIR_R &= ~0x01;                                            // make PG0 input (HiZ)
    
}

