#define _SUPPRESS_PLIB_WARNING

#include <plib.h>    
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#pragma config FPLLMUL = MUL_20, FPLLIDIV = DIV_2, FPLLODIV = DIV_1, FWDTEN = OFF
#pragma config POSCMOD = HS, FNOSC = PRIPLL, FPBDIV = DIV_1
#pragma config FSOSCEN = OFF

#define GetSystemClock()              (80000000ul)
#define GetPeripheralClock()          (GetSystemClock()/(1 << OSCCONbits.PBDIV))
#define GetInstructionClock()         (GetSystemClock())

#define DESIRED_BAUDRATE              (19200)  //The desired BaudRate
#define DAC_ADDRESS                   (0x10)  //Originalmente 0x10

void WriteString1(const char *string);
//void WriteString2(const char *string);
void msDelay(int Limite);
//void PrintSpeed(int rueda);
//void PrintChar(char msg);
void SendDAC(int send, int ref, int dac_selection);
void SetWheelData();
void setSpeed();

BOOL TransmitOneByte( UINT8 );
BOOL StartTransfer( BOOL );
void StopTransfer( void );

#define PI                            3.141592
#define vel_min                       0.2
#define vel_max                       2.047 //2.047
#define scale_factor                  111.79 //111.79
#define pulse_speed                   2

int i = 0;
char buffer[61], aux;
char enable = 0x0F;
char brake = 0x00;
char dir;
char sendFlag = 0;
char PacketReceive = 0;
char dribbler = 0;
char kick = 0
int speed[4], vPateo;
int id = 0;
float vel[3];
float kinematic[4][3];
int count = 0;
char corr = 0;

int main(void)
{
  SYSTEMConfig(GetSystemClock(), SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE);

  
  mPORTEClearBits(BIT_0,BIT_1,BIT_2,BIT_3,BIT_4,BIT_5,BIT_6,BIT_7);
  mPORTFClearBits(BIT_1);
  mPORTDClearBits(BIT_2,BIT_3,BIT_4);

  mPORTESetPinsDigitalOut(BIT_0,BIT_1,BIT_2,BIT_3,BIT_4,BIT_5,BIT_6,BIT_7);
  mPORTESetPinsDigitalOut(BIT_1);
  mPORTESetPinsDigitalOut(BIT_2,BIT_3,BIT_4);
  
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

  kinematic[0][0] = sin(-PI / 4.0); kinematic[0][1] = -cos(-PI / 4.0); kinematic[0][2] = -0.083648;
  kinematic[1][0] = sin(2.0 * (PI / 9.0)); kinematic[1][1] = -cos(2.0 * (PI / 9.0)); kinematic[1][2] = -0.083648;
  kinematic[2][0] = sin(7.0 * (PI / 9.0)); kinematic[2][1] = -cos(7.0 * (PI / 9.0)); kinematic[2][2] = -0.083648;
  kinematic[3][0] = sin(5.0 * (PI / 4.0)); kinematic[3][1] = -cos(5.0 * (PI / 4.0)); kinematic[3][2] = -0.083648;

  I2CSetFrequency(I2C2, GetPeripheralClock(), 100000);
  I2CEnable(I2C2, TRUE);
  SendDAC(0, 1, 0);
  
  msDelay(1000);  
  while (1)
  {
    if (sendFlag == 1)
    {        
        //setSpeed();
        SetWheelData();
        sendFlag = 0;
    }
    msDelay(1);
  }

  return 0;
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
void __ISR(_UART1_VECTOR, IPL2SOFT) IntUart1Handler(void)
{
  // Is this an RX interrupt?
  if (INTGetFlag(INT_SOURCE_UART_RX(UART1)))
    {
      char aux = (char) UARTGetDataByte(UART1);
      if (PacketReceive){ 
        buffer[i] = aux;

        if (buffer[i] == 'g'){
          i = 0;
          PacketReceive = 0;
          
          corr = 0;

          char vx_data[3];
          char vy_data[3];
          char vr_data[3];
          char drib_kick;
          
          vx_data[0] = buffer[0 + id * 9];
          vx_data[1] = buffer[1 + id * 9];
          vx_data[2] = buffer[2 + id * 9];

          vy_data[0] = buffer[3 + id * 9];
          vy_data[1] = buffer[4 + id * 9];
          vy_data[2] = buffer[5 + id * 9];

          vr_data[0] = buffer[6 + id * 9];
          vr_data[1] = buffer[7 + id * 9];
          vr_data[2] = buffer[8 + id * 9];
        
          drib_kick = buffer[9 + id * 9];

          int vel_temp = (int)strtol(vx_data, NULL, 16);
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
            speed[j] = (int)(fabs(vel_t) * scale_factor);
            
            if (speed[j] < 85 && speed[j] > 40) corr |= (1 << j);
          }
          sendFlag = 1;
          //SetWheelData();
        }

        else i++;
      }
      if (aux == 'h') PacketReceive = 1;
      // Clear the RX interrupt Flag
      INTClearFlag(INT_SOURCE_UART_RX(UART1));

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
          if(!I2CByteWasAcknowledged(I2C2))
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

void SetWheelData()
{
  // Frenos: se aplica a todas las ruedas
  if (brake && 0x0F) 
  {
    mPORTEClearBits(BIT_1 | BIT_4 | BIT_7);
    mPORTDClearBits(BIT_3);
  }
  else 
  {
    mPORTESetBits(BIT_1 | BIT_4 | BIT_7);
    mPORTDSetBits(BIT_3);
  }
  if (enable && 0x0F) 
  {
    mPORTESetBits(BIT_0 | BIT_3 | BIT_6);
    mPORTDSetBits(BIT_4);
  }
  else 
  {
    mPORTEClearBits(BIT_0 | BIT_3 | BIT_6);
    mPORTDClearBits(BIT_4);  
  }

  int k;
  for (k = 0; k < 4; k++) {
      if ((dir >> k) & 0x01) 
      {
          if (k == 0) {
              mPORTEClearBits(BIT_2);
          }
          else if (k == 1) {
              mPORTEClearBits(BIT_5);
          }
          else if (k == 2) {
              mPORTFClearBits(BIT_1);
          }
          else if (k == 3) {
              mPORTDClearBits(BIT_2);
          }
      }
      else 
      {
          if (k == 0) {
              mPORTESetBits(BIT_2);
          }
          else if (k == 1) {
              mPORTESetBits(BIT_5);
          }
          else if (k == 2) {
              mPORTFSetBits(BIT_1);
          }
          else if (k == 3) {
              mPORTDSetBits(BIT_2);
          }
      }
      /*
      if(dribbler) mPORTESetBits(BIT_6);
      else mPORTEClearBits(BIT_6);
      */
      /* 
      if(kick == 1) {
          mPORTESetBits(BIT_);
          mPORTEClearBits(BIT_);
      }
      else if (kick == 2) {
          mPORTESetBits(BIT_);
          mPORTEClearBits(BIT_);
      }
      else {
          mPORTEClearBits(BIT_);
          mPORTEClearBits(BIT_);
      }
      */    
      
      if((corr & (1 << k)) && !(corr && 0x0F)) speed[k] = 85;      
      SendDAC(speed[k], 0, k);
  }
}

BOOL StartTransfer( BOOL restart )
{
    I2C_STATUS  status;

    // Send the Start (or Restart) signal
    if(restart)
    {
        I2CRepeatStart(I2C2);
    }
    else
    {
        // Wait for the bus to be idle, then start the transfer
        while( !I2CBusIsIdle(I2C2) );

        if(I2CStart(I2C2) != I2C_SUCCESS)
        {
            DBPRINTF("Error: Bus collision during transfer Start\n");
            return FALSE;
        }
    }

    // Wait for the signal to complete
    do
    {
        status = I2CGetStatus(I2C2);

    } while ( !(status & I2C_START) );

    return TRUE;
}

BOOL TransmitOneByte( UINT8 data )
{
    // Wait for the transmitter to be ready
    while(!I2CTransmitterIsReady(I2C2));

    // Transmit the byte
    if(I2CSendByte(I2C2, data) == I2C_MASTER_BUS_COLLISION)
    {
        DBPRINTF("Error: I2C Master Bus Collision\n");
        return FALSE;
    }

    // Wait for the transmission to finish
    while(!I2CTransmissionHasCompleted(I2C2));

    return TRUE;
}

void StopTransfer( void )
{
    I2C_STATUS  status;

    // Send the Stop signal
    I2CStop(I2C2);

    // Wait for the signal to complete
    do
    {
        status = I2CGetStatus(I2C2);

    } while ( !(status & I2C_STOP) );
}

void setSpeed()
{
    int j, k;
    float vel_t;
    for (j = 0; j < 4; j++)
    {
		vel_t = 0;
		for (k = 0; k < 3; k++)
	    {
	    	vel_t += kinematic[j][k] * vel[k];
	    }

	    /*
	    // Signo y límite de la velocidad
	    if (fabs(vel_t) > vel_max)
	    {
		    if (vel_t < 0) dir |= 1 << j;
		    else dir &= ~(1 << j);
		    speed[j] = 400; // 400 por 1023
	    }
	    else if (fabs(vel_t) < vel_min)
	    {
		    if (vel_t < 0) dir |= 1 << j;
		    else dir &= ~(1 << j);
		    speed[j] = 0;
	    }
	    else
	    {
		    if (vel_t < 0) dir |= 1 << j;
		    else dir &= ~(1 << j);
		    speed[j] = fabs(vel_t) * 400 / vel_max; // 400 por 1023
	    }
	    */
	    //velocity[j] = vel_t;
	    if (vel_t < 0) dir &= ~(1 << j);
	    else dir |= 1 << j;
	    speed[j] = (int)(fabs(vel_t) * scale_factor);
    }
}
