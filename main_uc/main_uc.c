#define _SUPPRESS_PLIB_WARNING

#include <plib.h>     // Peripheral Library
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "nrf24l01.h"
#pragma config FPLLMUL = MUL_20, FPLLIDIV = DIV_2, FPLLODIV = DIV_1, FWDTEN = OFF
#pragma config POSCMOD = HS, FNOSC = PRIPLL, FPBDIV = DIV_1
#pragma config FSOSCEN = OFF

// *****************************************************************************
// *****************************************************************************
// Section: System Macros
// *****************************************************************************
// *****************************************************************************
#define GetSystemClock()              (80000000ul)
#define GetPeripheralClock()          (GetSystemClock()/*/(1 << OSCCONbits.PBDIV)*/)
#define GetInstructionClock()         (GetSystemClock())

#define DESIRED_BAUDRATE              (19200)  //The desired BaudRate
#define DAC_ADDRESS                   (0x10)  //Originalmente 0x10

void WriteString1(const char *string);
void msDelay(int Limite);
void PrintSpeed(int rueda);
void SetWheelData();
void SendDAC(int send, int ref, int dac_selection);
void Initialize(void);
void InitializeIO(void);
void SpiInitDevice(int, int, int, int);
void Delayus(unsigned);

BOOL TransmitOneByte( UINT8 );
BOOL StartTransfer( BOOL );
void StopTransfer( void );

#define PI              3.141592
#define vel_min         0.2
#define vel_max         2.047 //2.047

#define scale_factor    111.79 //111.79
#define pulse_speed     2

#define T3_TICK         GetPeripheralClock() / 256
#define MAX_DC          370
#define MIN_DC          10
#define Dt              0.0001
#define wheel_radio     0.0285
#define dac_scale       50.10612245 // 1023/20.4167 - Máximo valor DAC/Velocidad nominal motor

//Wheel config
#define INIT_ENABLE_1       {mPORTBClearBits(BIT_9); mPORTBSetPinsDigitalOut(BIT_9);}
#define INIT_ENABLE_2       {mPORTDClearBits(BIT_6); mPORTDSetPinsDigitalOut(BIT_6);}
#define INIT_ENABLE_3       {mPORTEClearBits(BIT_5); mPORTESetPinsDigitalOut(BIT_5);}
#define INIT_ENABLE_4       {mPORTBClearBits(BIT_1); mPORTBSetPinsDigitalOut(BIT_1);}
#define INIT_ENABLE         {INIT_ENABLE_1; INIT_ENABLE_2; INIT_ENABLE_3; INIT_ENABLE_4;}

#define SET_ENABLE_1        mPORTBSetBits(BIT_9)
#define SET_ENABLE_2        mPORTDSetBits(BIT_6)
#define SET_ENABLE_3        mPORTESetBits(BIT_5)
#define SET_ENABLE_4        mPORTBSetBits(BIT_1)

#define CLEAR_ENABLE_1      mPORTBClearBits(BIT_9)
#define CLEAR_ENABLE_2      mPORTDClearBits(BIT_6)
#define CLEAR_ENABLE_3      mPORTEClearBits(BIT_5)
#define CLEAR_ENABLE_4      mPORTBClearBits(BIT_1)

#define INIT_BRAKE_1        {mPORTBClearBits(BIT_14); mPORTBSetPinsDigitalOut(BIT_14);}
#define INIT_BRAKE_2        {mPORTEClearBits(BIT_0); mPORTESetPinsDigitalOut(BIT_0);}
#define INIT_BRAKE_3        {mPORTEClearBits(BIT_6); mPORTESetPinsDigitalOut(BIT_6);}
#define INIT_BRAKE_4        {mPORTBClearBits(BIT_3); mPORTBSetPinsDigitalOut(BIT_3);}
#define INIT_BRAKE          {INIT_BRAKE_1; INIT_BRAKE_2; INIT_BRAKE_3; INIT_BRAKE_4;}

#define SET_BRAKE_1         mPORTBSetBits(BIT_14)
#define SET_BRAKE_2         mPORTESetBits(BIT_0)
#define SET_BRAKE_3         mPORTESetBits(BIT_6)
#define SET_BRAKE_4         mPORTBSetBits(BIT_3)

#define CLEAR_BRAKE_1       mPORTBClearBits(BIT_14)
#define CLEAR_BRAKE_2       mPORTEClearBits(BIT_0)
#define CLEAR_BRAKE_3       mPORTEClearBits(BIT_6)
#define CLEAR_BRAKE_4       mPORTBClearBits(BIT_3)

#define INIT_FWDREV_1       {mPORTDClearBits(BIT_1); mPORTDSetPinsDigitalOut(BIT_1);}
#define INIT_FWDREV_2       {mPORTDClearBits(BIT_7); mPORTDSetPinsDigitalOut(BIT_7);}
#define INIT_FWDREV_3       {mPORTEClearBits(BIT_7); mPORTESetPinsDigitalOut(BIT_7);}
#define INIT_FWDREV_4       {mPORTBClearBits(BIT_2); mPORTBSetPinsDigitalOut(BIT_2);}
#define INIT_FWDREV         {INIT_FWDREV_1; INIT_FWDREV_2; INIT_FWDREV_3; INIT_FWDREV_4;}

#define INIT_WHEELS         {INIT_ENABLE; INIT_BRAKE; INIT_FWDREV}

#define SET_FWDREV_1        mPORTDSetBits(BIT_1)
#define SET_FWDREV_2        mPORTDSetBits(BIT_7)
#define SET_FWDREV_3        mPORTESetBits(BIT_7)
#define SET_FWDREV_4        mPORTBSetBits(BIT_2)

#define CLEAR_FWDREV_1      mPORTDClearBits(BIT_1)
#define CLEAR_FWDREV_2      mPORTDClearBits(BIT_7)
#define CLEAR_FWDREV_3      mPORTEClearBits(BIT_7)
#define CLEAR_FWDREV_4      mPORTBClearBits(BIT_2)

#define TOGGLE_FWDREV       {mPORTDToggleBits(BIT_1); mPORTDToggleBits(BIT_7); mPORTEToggleBits(BIT_7); mPORTBToggleBits(BIT_2);}

#define INIT_DRIBBLER       {mPORTEClearBits(BIT_3); mPORTESetPinsDigitalOut(BIT_3);}
#define SET_DRIBBLER        mPORTESetBits(BIT_3)
#define RELEASE_DRIBBLER    mPORTEClearBits(BIT_3)

#define INIT_KICK           {mPORTEClearBits(BIT_4); mPORTESetPinsDigitalOut(BIT_4);}
#define SET_KICK            mPORTESetBits(BIT_4)
#define RELEASE_KICK        mPORTEClearBits(BIT_4)

int i = 0;
char buffer[61], aux;

char enable = 0x0F;
char brake = 0x00;
char dir;

char sendFlag = 0;
char PacketReceive = 0;

char dribbler = 0;
char kick = 0;

int speed[4], vPateo;
int id = 0;

float vel[3];
float kinematic[4][3];
float ref_speed[4];

unsigned int count_caca = 0;

char corr = 0;
unsigned char width = 10;
//unsigned char data[32]; //register to hold letter sent and received
unsigned char data[10];
int count = 0;

int main(void)
{

  // Configure the device for maximum performance but do not change the PBDIV
  // Given the options, this function will change the flash wait states, RAM
  // wait state and enable prefetch cache but will not change the PBDIV.
  // The PBDIV value is already set via the pragma FPBDIV option above..
  SYSTEMConfig(GetSystemClock(), SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE);
  
  //DRIBBLER
  INIT_DRIBBLER;
  //KICK
  INIT_KICK;
  
  //WHEELS  
  INIT_ENABLE;
  INIT_BRAKE;
  INIT_FWDREV;

  SET_ENABLE_1;
  SET_ENABLE_2;
  SET_ENABLE_3;
  SET_ENABLE_4;

  SET_BRAKE_1;
  SET_BRAKE_2;
  SET_BRAKE_3;
  SET_BRAKE_4;  

  //SERIAL COMMUNICATION
  // Explorer-16 uses UART1 to connect to the PC.
  // This initialization assumes 36MHz Fpb clock. If it changes,
  // you will have to modify baud rate initializer.
  
  mPORTDClearBits(BIT_4 | BIT_5); 
  mPORTDSetPinsDigitalOut(BIT_4 | BIT_5);
  mPORTDSetBits(BIT_4 | BIT_5);
  
  UARTConfigure(UART1, UART_ENABLE_PINS_TX_RX_ONLY);
  UARTSetFifoMode(UART1, UART_INTERRUPT_ON_TX_NOT_FULL | UART_INTERRUPT_ON_RX_NOT_EMPTY);
  UARTSetLineControl(UART1, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
  UARTSetDataRate(UART1, GetPeripheralClock(), DESIRED_BAUDRATE);
  UARTEnable(UART1, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

  INTEnable(INT_SOURCE_UART_RX(UART1), INT_ENABLED);
  INTSetVectorPriority(INT_VECTOR_UART(UART1), INT_PRIORITY_LEVEL_2);
  INTSetVectorSubPriority(INT_VECTOR_UART(UART1), INT_SUB_PRIORITY_LEVEL_0);
  
  INTConfigureSystem(INT_SYSTEM_CONFIG_MULT_VECTOR);

  INTEnableInterrupts();
  
  I2CSetFrequency(I2C1, GetPeripheralClock(), 100000);
  I2CEnable(I2C1, TRUE);
  SendDAC(0, 1, 0);
  
  ref_speed[0] = 0.0; ref_speed[1] = 0.0; ref_speed[2] = 0.0; ref_speed[3] = 0.0;
  vel[0] = 0.0; vel[1] = 0.0; vel[2] = 0.0;

  kinematic[0][0] = sin(-PI / 4.0); kinematic[0][1] = -cos(-PI / 4.0); kinematic[0][2] = -0.083648;
  kinematic[1][0] = sin(2.0 * (PI / 9.0)); kinematic[1][1] = -cos(2.0 * (PI / 9.0)); kinematic[1][2] = -0.083648;
  kinematic[2][0] = sin(7.0 * (PI / 9.0)); kinematic[2][1] = -cos(7.0 * (PI / 9.0)); kinematic[2][2] = -0.083648;
  kinematic[3][0] = sin(5.0 * (PI / 4.0)); kinematic[3][1] = -cos(5.0 * (PI / 4.0)); kinematic[3][2] = -0.083648;
  
  //Initialize(); //initialize IO, UART, SPI, set up nRF24L01 as TX
  //unsigned char nrf_status;
  // Let interrupt handler do the work
  while (1){
      msDelay(5000);
      SET_DRIBBLER;
      msDelay(5000);
      RELEASE_DRIBBLER;
      /*
        nrf24l01_irq_clear_all(); //clear interrupts again
        
        while(!(nrf24l01_irq_pin_active() && nrf24l01_irq_rx_dr_active())){
            nrf_status = nrf24l01_get_status();
        }

        nrf24l01_read_rx_payload(data, width); //get the payload into data
        nrf24l01_irq_clear_all(); //clear interrupts again
        msDelay(1);
        
        char vx_data[8];
        char vy_data[8];
        char vr_data[8];
        char drib_kick;

        vx_data[0] = data[0 + id * 10];
        vx_data[1] = data[1 + id * 10];
        vx_data[2] = data[2 + id * 10];
        vx_data[3] = 0;
        vx_data[4] = 0;
        vx_data[5] = 0;
        vx_data[6] = 0;
        vx_data[7] = 0;

        vy_data[0] = data[3 + id * 10];
        vy_data[1] = data[4 + id * 10];
        vy_data[2] = data[5 + id * 10];
        vy_data[3] = 0;
        vy_data[4] = 0;
        vy_data[5] = 0;
        vy_data[6] = 0;
        vy_data[7] = 0;

        vr_data[0] = data[6 + id * 10];
        vr_data[1] = data[7 + id * 10];
        vr_data[2] = data[8 + id * 10];
        vr_data[3] = 0;
        vr_data[4] = 0;
        vr_data[5] = 0;
        vr_data[6] = 0;
        vr_data[7] = 0;

        drib_kick = data[9 + id * 10];

        int vel_temp = 0;

        vel_temp = (int)strtol(vx_data, NULL, 16);
        if (vel_temp & 0x800) vel_temp = -((vel_temp ^ 0xFFF) + 1);

        vel[0] = vel_temp / 1000.0;

        vel_temp = (int)strtol(vy_data, NULL, 16);
        if (vel_temp & 0x800) vel_temp = -((vel_temp ^ 0xFFF) + 1);

        vel[1] = vel_temp / 1000.0;

        vel_temp = (int)strtol(vr_data, NULL, 16);
        if (vel_temp & 0x800) vel_temp = -((vel_temp ^ 0xFFF) + 1);

        vel[2] = vel_temp / 1000.0;
        dribbler = (drib_kick & 0x04) >> 2;
        kick = drib_kick & 0x03;
        //if (vel[1] &)          
        int j, k;
        float vel_t;
        for (j = 0; j < 4; j++)
        {
          vel_t = 0;
          for (k = 0; k < 3; k++)
          {
            vel_t += kinematic[j][k] * vel[k];
          }

          if (vel_t < 0) dir &= ~(1 << j);
          else dir |= 1 << j;

          ref_speed[j] = ((fabs(vel_t) / wheel_radio) * (2.4 / (2*M_PI)));
          count_caca = 1;
        }
        SetWheelData();
      
        */
    
        if (count_caca == 1){
            SetWheelData();
            count_caca = 0;
        }
    }

    return 0;
}

//initialize routine
void Initialize()
{
	InitializeIO(); //set up IO (directions and functions)
	SpiInitDevice(2, 1, 0, 0);
	nrf24l01_initialize_debug(true, width, false); //initialize the 24L01 to the debug configuration as RX, 1 data byte, and auto-ack disabled
}

//initialize IO pins
void InitializeIO(void)
{
	mPORTGClearBits(BIT_9);
	mPORTDClearBits(BIT_5 | BIT_8);

	mPORTDSetPinsDigitalIn(BIT_8);

	mPORTGSetPinsDigitalOut(BIT_9);
	mPORTDSetPinsDigitalOut(BIT_5);
  	//LATG |= BIT_9; // set CSN bit active
}

unsigned char spi_send_read_byte(unsigned char byte) {
	unsigned short	txData, rxData; // transmit, receive characters
	int chn = 2; // SPI channel to use (1 or 2)
	 
	txData = byte; // take inputted byte and store into txData   
	SpiChnPutC(chn, txData);			// send data
	rxData = SpiChnGetC(chn);			// retreive over channel chn the received data into rxData

	return rxData; 
}

void SpiInitDevice(int chn, int isMaster, int frmEn, int frmMaster) {
	unsigned int config = SPI_CON_MODE8 | SPI_CON_SMP | SPI_CON_ON;	// SPI configuration word
	if(isMaster) {
	 	config |= SPI_CON_MSTEN;
	}
	if(frmEn) {
		config |= SPI_CON_FRMEN;
		if(!frmMaster) {   
			config |= SPI_CON_FRMSYNC;
  		}
	}
	SpiChnOpen(chn, config, 4);	// divide fpb by 4, configure the I/O ports. Not using SS in this example
}

// Delayus
void Delayus( unsigned t) {
	OpenTimer1(T1_ON | T1_PS_1_256, 0xFFFF);
	while (t--)	{  // t x 1ms loop
		WriteTimer1(0);
		while (ReadTimer1() < GetSystemClock()/256/1000000);
	}
	CloseTimer1();
} 

// helper functions
void WriteString1(const char *string)
{
  while (*string != '\0')
    {
      while (!UARTTransmitterIsReady(UART1));

      UARTSendDataByte(UART1, *string);

      string++;

      while (!UARTTransmissionHasCompleted(UART1));
    }
}

void PutCharacter(const char character)
{
  while (!UARTTransmitterIsReady(UART1));

  UARTSendDataByte(UART1, character);

  while (!UARTTransmissionHasCompleted(UART1));
}

// UART 1 interrupt handler
// it is set at priority level 2 with software context saving
void __ISR(_UART1_VECTOR, IPL4SOFT) IntUart1Handler(void)
{
  // Is this an RX interrupt?
  if (INTGetFlag(INT_SOURCE_UART_RX(UART1)))
    {
      // Clear the RX interrupt Flag
      INTClearFlag(INT_SOURCE_UART_RX(UART1));
      char aux = (char) UARTGetDataByte(UART1);
      if (PacketReceive){ 
        buffer[i] = aux;

        if (buffer[i] == 'g'){
          i = 0;
          PacketReceive = 0;
          
          corr = 0;

          char vx_data[8];
          char vy_data[8];
          char vr_data[8];
          char drib_kick;
          
          vx_data[0] = buffer[0 + id * 10];
          vx_data[1] = buffer[1 + id * 10];
          vx_data[2] = buffer[2 + id * 10];
          vx_data[3] = 0;
          vx_data[4] = 0;
          vx_data[5] = 0;
          vx_data[6] = 0;
          vx_data[7] = 0;

          vy_data[0] = buffer[3 + id * 10];
          vy_data[1] = buffer[4 + id * 10];
          vy_data[2] = buffer[5 + id * 10];
          vy_data[3] = 0;
          vy_data[4] = 0;
          vy_data[5] = 0;
          vy_data[6] = 0;
          vy_data[7] = 0;

          vr_data[0] = buffer[6 + id * 10];
          vr_data[1] = buffer[7 + id * 10];
          vr_data[2] = buffer[8 + id * 10];
          vr_data[3] = 0;
          vr_data[4] = 0;
          vr_data[5] = 0;
          vr_data[6] = 0;
          vr_data[7] = 0;
        
          drib_kick = buffer[9 + id * 10];
          
          int vel_temp = 0;
          
          vel_temp = (int)strtol(vx_data, NULL, 16);
          if (vel_temp & 0x800) vel_temp = -((vel_temp ^ 0xFFF) + 1);
          
          vel[0] = vel_temp / 1000.0;

          vel_temp = (int)strtol(vy_data, NULL, 16);
          if (vel_temp & 0x800) vel_temp = -((vel_temp ^ 0xFFF) + 1);

          vel[1] = vel_temp / 1000.0;

          vel_temp = (int)strtol(vr_data, NULL, 16);
          if (vel_temp & 0x800) vel_temp = -((vel_temp ^ 0xFFF) + 1);

          vel[2] = vel_temp / 1000.0;
          dribbler = (drib_kick & 0x04) >> 2;
          kick = drib_kick & 0x03;
          //if (vel[1] &)          
          int j, k;
          float vel_t;
          for (j = 0; j < 4; j++)
          {
            vel_t = 0;
            for (k = 0; k < 3; k++)
            {
              vel_t += kinematic[j][k] * vel[k];
            }

            if (vel_t < 0) dir &= ~(1 << j);
            else dir |= 1 << j;
            
            ref_speed[j] = ((fabs(vel_t) / wheel_radio) * (1.0 / (2*M_PI)));
            count_caca = 1;
          }
        }

        else i++;
      }
      if (aux == 'h') PacketReceive = 1;
    }

  // We don't care about TX interrupt
  if ( INTGetFlag(INT_SOURCE_UART_TX(UART1)) )
    {
      INTClearFlag(INT_SOURCE_UART_TX(UART1));
    }
}

void msDelay(int Limite)
{
  int cantidad = 0;
  int ms = 0;
  for(cantidad = 0; cantidad  < Limite; cantidad++){
        
    for(ms = 0; ms < 4500; ms++){
      asm("NOP");
      asm("NOP");
      asm("NOP");
      asm("NOP");
      asm("NOP");
      asm("NOP");
      asm("NOP");
      asm("NOP");
      asm("NOP");
      asm("NOP");
    }
  }
}


void SetWheelData()
{  
    int k;
    for (k = 3; k >= 0; k--) {
        if ((dir >> k) & 0x01) {
            if (k == 0) {
                CLEAR_FWDREV_1;
            }
            else if (k == 1) {
                CLEAR_FWDREV_2;
            }
            else if (k == 2) {
                CLEAR_FWDREV_3;
            }
            else if (k == 3) {
                CLEAR_FWDREV_4;
            }
        }
        else {
            if (k == 0) {
                SET_FWDREV_1;
            }
            else if (k == 1) {
                SET_FWDREV_2;
            }
            else if (k == 2) {
                SET_FWDREV_3;
            }
            else if (k == 3) {
                SET_FWDREV_4;
            }
        }
        //ref_speed[k]*1000/100
        if (k == 0){
            if (ref_speed[k] == 0.0)
                CLEAR_ENABLE_1;
            else
                SET_ENABLE_1;
        }
        if (k == 1){
            if (ref_speed[k] == 0.0)
                CLEAR_ENABLE_2;
            else
                SET_ENABLE_2;
        }
        if (k == 2){
            if (ref_speed[k] == 0.0)
                CLEAR_ENABLE_3;
            else
                SET_ENABLE_3;
        }
        if (k == 3){
            if (ref_speed[k] == 0.0)
                CLEAR_ENABLE_4;
            else
                SET_ENABLE_4;
        }
        SendDAC((int)(ref_speed[k]*dac_scale), 0, k);
    }
    
    if (dribbler) 
        SET_DRIBBLER;
    else 
        RELEASE_DRIBBLER;
    
    if (kick) 
        SET_KICK;
    else 
        RELEASE_DRIBBLER;
}

void SendDAC(int send, int ref, int dac_selection)
{
    UINT8               i2cData[10];
    I2C_7_BIT_ADDRESS   SlaveAddress;
    int                 Index;
    int                 DataSz;
    UINT32              actualClock;
    BOOL                Acknowledged;
    BOOL                Success = TRUE;
    UINT8               i2cbyte;

    // OJO DAC_ADDRESS, se cambia a 0x11
    I2C_FORMAT_7_BIT_ADDRESS(SlaveAddress, DAC_ADDRESS, I2C_WRITE);
    if(ref == 1){
        i2cData[0] = SlaveAddress.byte;
        i2cData[1] = 0x76;//Internal Ref 2.0V
        i2cData[2] = 0x40;//Don't Care
        i2cData[3] = 0xAA;//Don't Care
    }
    else {
        i2cData[0] = SlaveAddress.byte;
        i2cData[1] = dac_selection;
        i2cData[2] = (send & 1020) >> 2;
        i2cData[3] = (send & 3) << 6;
    }
    DataSz = 4;    

    if( !StartTransfer(FALSE) )
    {
        while(1);
    }
        // Transmit all data
    Index = 0;
    while( Success && (Index < DataSz) )
    {
        // Transmit a byte
        if (TransmitOneByte(i2cData[Index]))
        {
            // Advance to the next byte
            Index++;

            // Verify that the byte was acknowledged
            if(!I2CByteWasAcknowledged(I2C1))
            {
                DBPRINTF("Error: Sent byte was not acknowledged\n");
                Success = FALSE;
            }
        }
        else
        {
            Success = FALSE;
        }
    }

    // End the transfer (hang here if an error occured)
    StopTransfer();
    if(!Success)
    {
        while(1);
    }
}


BOOL StartTransfer( BOOL restart )
{
    I2C_STATUS  status;

    // Send the Start (or Restart) signal
    if(restart)
    {
        I2CRepeatStart(I2C1);
    }
    else
    {
        // Wait for the bus to be idle, then start the transfer
        while( !I2CBusIsIdle(I2C1) );

        if(I2CStart(I2C1) != I2C_SUCCESS)
        {
            DBPRINTF("Error: Bus collision during transfer Start\n");
            return FALSE;
        }
    }

    // Wait for the signal to complete
    do
    {
        status = I2CGetStatus(I2C1);

    } while ( !(status & I2C_START) );

    return TRUE;
}

BOOL TransmitOneByte( UINT8 data )
{
    // Wait for the transmitter to be ready
    while(!I2CTransmitterIsReady(I2C1));

    // Transmit the byte
    if(I2CSendByte(I2C1, data) == I2C_MASTER_BUS_COLLISION)
    {
        DBPRINTF("Error: I2C Master Bus Collision\n");
        return FALSE;
    }

    // Wait for the transmission to finish
    while(!I2CTransmissionHasCompleted(I2C1));

    return TRUE;
}

void StopTransfer( void )
{
    I2C_STATUS  status;

    // Send the Stop signal
    I2CStop(I2C1);

    // Wait for the signal to complete
    do
    {
        status = I2CGetStatus(I2C1);

    } while ( !(status & I2C_STOP) );
}

void PrintSpeed(int rueda)
{
  char strSpeed[6];
  
  if (((dir >> rueda) & 0x1) == 1) strSpeed[0] = '-';
  else strSpeed[0] = '+';
  
  /*
  strSpeed[1] = (char) (RPM[rueda] / 1000 + 48);
  strSpeed[2] = (char) ((RPM[rueda] % 1000) / 100 + 48);
  strSpeed[3] = (char) (((RPM[rueda] % 1000) % 100) / 10) + 48;
  strSpeed[4] = (char) (((RPM[rueda] % 1000) % 100) % 10) + 48;
  strSpeed[5] = '\0';
  */
  WriteString1(strSpeed);
}
