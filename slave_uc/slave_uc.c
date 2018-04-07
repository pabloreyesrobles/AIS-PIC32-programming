#define _SUPPRESS_PLIB_WARNING

#include <plib.h>     // Peripheral Library
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#pragma config FPLLMUL = MUL_20, FPLLIDIV = DIV_2, FPLLODIV = DIV_1, FWDTEN = OFF
#pragma config POSCMOD = HS, FNOSC = PRIPLL, FPBDIV = DIV_8
#pragma config FSOSCEN = OFF

// *****************************************************************************
// *****************************************************************************
// Section: System Macros
// *****************************************************************************
// *****************************************************************************
#define GetSystemClock()              (80000000ul)
#define GetPeripheralClock()          (GetSystemClock()/(1 << OSCCONbits.PBDIV))
#define GetInstructionClock()         (GetSystemClock())

#define DESIRED_BAUDRATE              (19200)  //The desired BaudRate
#define DAC_ADDRESS                   (0x10)  //Originalmente 0x10

void WriteString2(const char *string);
void msDelay(int Limite);

//void PrintChar(char msg);
void SetWheelData();
BOOL TransmitOneByte( UINT8 );
BOOL StartTransfer( BOOL );
void StopTransfer( void );

#define PI                            3.141592
#define vel_min                       0.2
#define vel_max                       2.047 //2.047

#define pulse_speed                   2

#define T3_TICK                       GetPeripheralClock() / 256
#define MAX_DC                        370
#define MIN_DC                        10
#define Dt                            0.0001
#define robot_radio                   0.0285

//#define CONTROL_MEASURE

static unsigned short int t1_new; // Captura de registro TMR3
static unsigned short int t1_old = 0; // Captura previa del registro TMR3
static unsigned short int time1_diff; // Diferencia entre capturas

float RPS;
float ref_speed;
int RPM;
int DC;

float error;
float error_prev;
float integral;
float output;

int time4_buf;

int tick1 = 0;

float Kp = 30.0, Ki = 120.0, Kd = 1.0;

int count = 0;

int main(void)
{

  // Configure the device for maximum performance but do not change the PBDIV
  // Given the options, this function will change the flash wait states, RAM
  // wait state and enable prefetch cache but will not change the PBDIV.
  // The PBDIV value is already set via the pragma FPBDIV option above..
  SYSTEMConfig(GetSystemClock(), SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE);
  
  #ifdef CONTROL_MEASURE //TIMER 3: MODO CAPTURA PARA MEDICIÓN

  OpenTimer3(T3_ON | T3_PS_1_256 | T3_SOURCE_INT, T3_TICK >> 1);
  ConfigIntTimer3(T3_INT_ON | T3_INT_PRIOR_2 | T3_INT_SUB_PRIOR_2);

  mIC1ClearIntFlag();
  
  OpenCapture1(IC_EVERY_FALL_EDGE | IC_INT_1CAPTURE | IC_TIMER3_SRC | IC_FEDGE_FALL | IC_CAP_16BIT | IC_ON);
  ConfigIntCapture1(IC_INT_ON | IC_INT_PRIOR_3 | IC_INT_SUB_PRIOR_0);

  #endif

  //TIMER 2: PWM
  PORTSetPinsDigitalOut(IOPORT_D, BIT_4);

  //EL RELOJ PBCLK SE DIVIDE POR 4 PARA GENERAR UNA PWM DE 10000HZ
  OpenTimer2(T2_ON | T2_PS_1_4 | T2_SOURCE_INT, 1000);
  //ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_2 | T2_INT_SUB_PRIOR_1);

  OpenOC5(OC_ON | OC_TIMER_MODE16 | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE, 0, 0);

  INTConfigureSystem(INT_SYSTEM_CONFIG_MULT_VECTOR);

  INTEnableInterrupts();

  RPS = 0;
  DC = 0;
  ref_speed = 2.0;

  DC = 500;
   
  SetDCOC5PWM(DC);
  
  // Let interrupt handler do the work
  while (1){
      //Nada por ahora
  }

  return 0;
}

// helper functions
void WriteString2(const char *string)
{
  while (*string != '\0')
    {
      while (!UARTTransmitterIsReady(UART2));

      UARTSendDataByte(UART2, *string);

      string++;

      while (!UARTTransmissionHasCompleted(UART2));
    }
}

void PutCharacter(const char character)
{
  while (!UARTTransmitterIsReady(UART2));

  UARTSendDataByte(UART2, character);

  while (!UARTTransmissionHasCompleted(UART2));
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
  DC = (int)((ref_speed * 1000) / 65.0);
  SetDCOC5PWM(DC);  
}

#ifdef CONTROL_MEASURE // INTERRUPCIONES DE CONTROL

void __ISR( _INPUT_CAPTURE_1_VECTOR, IPL3SOFT) Capture1(void)//RD8
{ 
    static unsigned int con1_buf; 

    ReadCapture1(con1_buf); // Read captures into buffer
    t1_new = con1_buf; // Save time of event
    time1_diff = t1_new - t1_old; // Compute elapsed time in timer â??ticksâ?
    t1_old = t1_new;  // Replace previous time capture with new
    
    RPS = (int)(T3_TICK / (8 * time1_diff * 2.4));
    
    //  Compute motor speed in RPS[ ](revolutions per second) and save as global variable
    //  Details left as an exercise for the Reader 
    mIC1ClearIntFlag(); // Clears interrupt flag
    // INTClearFlag(INT_IC1); // Alternate peripheral library function
}


void __ISR( _TIMER_2_VECTOR, IPL2SOFT) T2Interrupt(void)
{
    
    error = ref_speed - RPS;
    integral += error * Dt;
    
    output = Kp * error + Ki * integral + Kd * (error - error_prev) / Dt;
    
    error_prev = error;
    
    DC += (int)output;
    
    if (DC > MAX_DC) DC = MAX_DC;
    
    if (DC < 0) DC = 0;
    //SetDCOC2PWM(1406);
    //SetDCOC3PWM(22500);
    //SetDCOC4PWM(1406);
    SetDCOC5PWM(DC);
    mT2ClearIntFlag();
} 

#endif