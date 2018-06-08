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

void WriteString1(const char *string);
void msDelay(int Limite);
void PrintSpeed(float rueda);
void SetWheelData();

BOOL TransmitOneByte( UINT8 );
BOOL StartTransfer( BOOL );
void StopTransfer( void );

#define PI                            3.141592
#define vel_min                       0.2
#define vel_max                       2.047 //2.047

#define scale_factor                  111.79 //111.79
#define pulse_speed                   2

#define T3_TICK                       GetPeripheralClock() / 256.0
#define T2_TICK                       GetPeripheralClock() / 256.0
#define MAX_DC                        500
#define MIN_DC                        10
#define Dt                            0.0016
#define robot_radio                   0.0285
#define analog_scale                  1.65                          // 1023/620 - M�ximo valor DAC/M�ximo valor ADC PIC32 PID
#define speed_scale                   50.10612245                   // 1023/20.4167 - M�ximo valor DAC/Velocidad nominal RUEDA
#define DC_scale                      0.488748553                   // 500/1023 - M�ximo valor DAC/Velocidad nominal motor

unsigned int count_caca = 0;

int i = 0;
char buffer[10], aux;
int PacketReceive = 0;

char corr = 0;

volatile unsigned int t_new; // Captura de registro TMR3
volatile unsigned int t_old = 0; // Captura previa del registro TMR3
volatile unsigned int time_diff; // Diferencia entre capturas

volatile float RPS;
volatile float ref_speed;
int RPM;
volatile float DC;

volatile float error = 0;
volatile float error_prev = 0;
volatile float integral = 0;
volatile float output = 0;

float Kp = 0.31, Ki = 0.35, Kd = 0.0006; // Ajustar

volatile int count = 0;
unsigned int channel = 0;

int main(void)
{
    // Configure the device for maximum performance but do not change the PBDIV
    // Given the options, this function will change the flash wait states, RAM
    // wait state and enable prefetch cache but will not change the PBDIV.
    // The PBDIV value is already set via the pragma FPBDIV option above..
    SYSTEMConfig(GetSystemClock(), SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE);

    //TIMER 3: MODO CAPTURA PARA MEDICIÃN
    OpenTimer3(T3_ON | T3_PS_1_256 | T3_SOURCE_INT, T3_TICK);
    //ConfigIntTimer3(T3_INT_ON | T3_INT_PRIOR_1 | T3_INT_SUB_PRIOR_2);

    mPORTDClearBits(BIT_11); 
    mPORTDSetPinsDigitalIn(BIT_11);
    mIC4ClearIntFlag();  

    OpenCapture4(IC_EVERY_RISE_EDGE | IC_INT_1CAPTURE | IC_TIMER3_SRC | IC_FEDGE_FALL | IC_CAP_16BIT | IC_ON);
    ConfigIntCapture4(IC_INT_ON | IC_INT_PRIOR_3 | IC_INT_SUB_PRIOR_0);
    

    /*
    UARTConfigure(UART1, UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(UART1, UART_INTERRUPT_ON_TX_NOT_FULL | UART_INTERRUPT_ON_RX_NOT_EMPTY);
    UARTSetLineControl(UART1, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    UARTSetDataRate(UART1, GetPeripheralClock(), DESIRED_BAUDRATE);
    UARTEnable(UART1, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

    INTEnable(INT_SOURCE_UART_RX(UART1), INT_ENABLED);
    INTSetVectorPriority(INT_VECTOR_UART(UART1), INT_PRIORITY_LEVEL_2);
    INTSetVectorSubPriority(INT_VECTOR_UART(UART1), INT_SUB_PRIORITY_LEVEL_0);
    */
    //ADC
    // configure and enable the ADC
    CloseADC10();	// ensure the ADC is off before setting the configuration

	// define setup parameters for OpenADC10
	// 				Turn module on | ouput in integer | trigger mode auto | enable autosample
	#define PARAM1  ADC_FORMAT_INTG | ADC_CLK_AUTO | ADC_AUTO_SAMPLING_ON

	// define setup parameters for OpenADC10
	// 				ADC ref external    | disable offset test    | disable scan mode | perform 1 samples | use dual buffers | use alternate mode
	#define PARAM2  ADC_VREF_AVDD_AVSS | ADC_OFFSET_CAL_DISABLE | ADC_SCAN_OFF | ADC_SAMPLES_PER_INT_1 | ADC_ALT_BUF_ON | ADC_ALT_INPUT_ON

	// define setup parameters for OpenADC10
	//				  use ADC internal clock | set sample time
	#define PARAM3  ADC_CONV_CLK_INTERNAL_RC | ADC_SAMPLE_TIME_15

	// define setup parameters for OpenADC10
	//               set AN1 
	#define PARAM4	ENABLE_AN1_ANA

	// define setup parameters for OpenADC10
	// do not assign channels to scan
	#define PARAM5	SKIP_SCAN_ALL
	
	// use ground as neg ref for A | use AN1 for input A 

	// configure to sample AN1
	SetChanADC10( ADC_CH0_NEG_SAMPLEA_NVREF | ADC_CH0_POS_SAMPLEA_AN1 ); // configure to sample AN1
	OpenADC10( PARAM1, PARAM2, PARAM3, PARAM4, PARAM5 ); // configure ADC using the parameters defined above

	EnableADC10(); // Enable the ADC
  
    //TIMER 2: PWM
    mPORTDClearBits(BIT_1); 
    mPORTDSetPinsDigitalOut(BIT_1);
    RPS = 0;
    DC = 0;
    ref_speed = 5.0;
    //EL RELOJ PBCLK SE DIVIDE POR 4 PARA GENERAR UNA PWM DE 10000HZ
    OpenTimer2(T2_ON | T2_PS_1_16 | T2_SOURCE_INT, 1000);
    ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_2 | T2_INT_SUB_PRIOR_1);

    OpenOC2(OC_ON | OC_TIMER_MODE16 | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE, 0, 0);

    //Debug
    mPORTDClearBits(BIT_9 | BIT_10); 
    mPORTDSetPinsDigitalOut(BIT_9 | BIT_10);

    INTConfigureSystem(INT_SYSTEM_CONFIG_MULT_VECTOR);

    INTEnableInterrupts();


    // Let interrupt handler do the work
    while (1){   
        
        while(!mAD1GetIntFlag() ) 
        { 

        }

        channel = ReadADC10(0);
        //DC = channel*analog_scale*DC_scale;
        ref_speed = channel*analog_scale/speed_scale;
        //msDelay(10);
        mAD1ClearIntFlag();
        
    }

    return 0;
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
    // En vez de 100.0 había un 65.0 Pablo del futuro
    DC = (int)((ref_speed * 1000) / 65.0);
    SetDCOC2PWM(DC); 

    if(DC > MAX_DC) DC = MAX_DC;
}

void __ISR( _INPUT_CAPTURE_4_VECTOR, IPL3AUTO) Capture4(void)
{
    unsigned int con_buf[4]; 
    
    ReadCapture4(con_buf); // Read captures into buffer
    t_new = con_buf[0] & 0x0000FFFF; // Save time of event
    //t_new = TMR3;
    time_diff = t_new - t_old; // Compute elapsed time in timer Ã¢ÂÂticksÃ¢ÂÂ
    t_old = t_new;  // Replace previous time capture with new
    
    RPS = (T3_TICK / (6 * time_diff * 2.4));
    count = 0;
    //  Compute motor speed in RPS[ ](revolutions per second) and save as global variable
    //  Details left as an exercise for the Reader
    mIC4ClearIntFlag(); // Clears interrupt flag
    // INTClearFlag(INT_IC4); // Alternate peripheral library function
}

void __ISR( _TIMER_2_VECTOR, IPL2AUTO) T2Interrupt(void)
{
    if(RPS > 21.0) RPS = 21.0;
    mPORTDSetBits(BIT_9);
    error = ref_speed - RPS;
    integral += error*Dt;
    
    output = Kp*error + Ki*integral + Kd*(error - error_prev)/Dt;
    
    count++;
    error_prev = error;
    
    DC += output;
    
    if (DC > MAX_DC) DC = MAX_DC;
    
    if (DC < 0) DC = 0;
    
    SetDCOC2PWM(DC);
    /*
    DC = (int)(channel*analog_scaling);
    SetDCOC2PWM(DC);
    
    WriteString1((char *)"Ref: ");
    PrintSpeed(ref_speed*100);
    WriteString1((char *)" - ");
    */
    mPORTDClearBits(BIT_9);
    mT2ClearIntFlag();
}

void PrintSpeed(float rueda)
{
    char strSpeed[6];

    strSpeed[0] = (char) ((int)rueda / 1000 + 48);
    strSpeed[1] = (char) (((int)rueda % 1000) / 100 + 48);
    strSpeed[2] = (char) '.';
    strSpeed[3] = (char) ((((int)rueda % 1000) % 100) / 10) + 48;
    strSpeed[4] = (char) ((((int)rueda % 1000) % 100) % 10) + 48;
    strSpeed[5] = '\0';

    WriteString1(strSpeed);
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
        // Clear the RX interrupt Flag
        INTClearFlag(INT_SOURCE_UART_RX(UART1));
        char aux = (char) UARTGetDataByte(UART1);
        if (PacketReceive){ 
            buffer[i] = aux;

            if (buffer[i] == 'g'){
              i = 0;
              int pid_selector = PacketReceive;
              PacketReceive = 0;

              corr = 0;

              char pid_set[8];

              pid_set[0] = buffer[0];
              pid_set[1] = buffer[1];
              pid_set[2] = buffer[2];
              pid_set[3] = buffer[3];
              pid_set[4] = buffer[4];
              pid_set[5] = buffer[5];
              pid_set[6] = buffer[6];
              pid_set[7] = buffer[7];

              unsigned int pid_tunning = 0;

              pid_tunning = (unsigned int)strtoul(pid_set, NULL, 10);

              if (pid_selector == 1) Kp = pid_tunning / 10000.0;
              if (pid_selector == 2) Ki = pid_tunning / 10000.0;
              if (pid_selector == 3) Kd = pid_tunning / 10000.0;
              corr = 1;
            }

            else i++;
        }
        if (aux == 'p') PacketReceive = 1;
        if (aux == 'i') PacketReceive = 2;
        if (aux == 'd') PacketReceive = 3;
    }

    // We don't care about TX interrupt
    if ( INTGetFlag(INT_SOURCE_UART_TX(UART1)) )
    {
        INTClearFlag(INT_SOURCE_UART_TX(UART1));
    }
}
