#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/pwm.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "driverlib/adc.h"
#include "filters.h"
#include "pid.h"


//*****************************************************************************

//*****************************************************************************
//
// Flags that contain the current value of the interrupt indicator as displayed
// on the UART.
//
//*****************************************************************************
uint32_t g_ui32Flags;

//*****************************************************************************
//
// The variable g_ui32PWMIncrement contains the value needed to increment the
// PWM duty cycle by 0.1% cycles based on the System Clock speed.
//
//*****************************************************************************
int32_t g_ui32PWMIncrement;
//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif


#define TEMPERATURE_ADC_VALUE   2000//3300    //ADC value for 65 degrees - Rseries = 100k Ohm
#define TEMPERATURE_OFFSET 5
#define KP  10.0F //10.0F
#define KI  0.01F //0.01F
#define KD  0.0F

/* GlobVars */
uint16_t setPoint = 65 + TEMPERATURE_OFFSET;  //65 + 4 (offset)
PIDController_t pid;
int32_t heater_PWM_lastone = 0;
uint16_t PIDfirst_time=0;
uint16_t flagPID=0;

uint8_t goPD=0;

void measureADC()
{

    uint32_t pui32ADC8Value[4];//uint32_t pui32ADC0Value[4];
    uint16_t ReadingArray1[filterSamples];
    uint16_t ReadingArray2[filterSamples];
    uint16_t ReadingArray3[filterSamples];
    uint16_t ReadingArray4[filterSamples];
    uint32_t aux1,aux2,aux3,aux4;
    uint8_t i=0;


    for (i = 0; i < filterSamples; i++)
      {

        // Trigger the ADC conversion.
        ADCProcessorTrigger(ADC1_BASE, 1); //3
        // Wait for conversion to be completed.
        while (!ADCIntStatus(ADC1_BASE, 1, false))//3
        {
        }
        // Clear the ADC interrupt flag.
        ADCIntClear(ADC1_BASE, 1);

        ADCSequenceDataGet(ADC1_BASE, 1, pui32ADC8Value);

        ReadingArray1[i] = pui32ADC8Value[0];
        ReadingArray2[i] = pui32ADC8Value[1];
        //ReadingArray3[i] = pui32ADC8Value[2];
        //ReadingArray4[i] = pui32ADC8Value[3];

        SysCtlDelay(SysCtlClockGet() / 1200);

      }

    aux1 = SmoothData(ReadingArray1,filterSamples);
    aux2 = SmoothData(ReadingArray2,filterSamples);
    //aux3 = SmoothData(ReadingArray3,filterSamples);
    //aux4 = SmoothData(ReadingArray4,filterSamples);

#ifdef ADC
  //UARTprintf("2000,");
    UARTprintf("%d,", aux1);
    UARTprintf("%d\n", aux2);
    //UARTprintf("%d,", aux3);
    //UARTprintf("%d\n", aux4);

#endif

}



void initPID(){

    /* initialize PID terms for heater */
    pid.Kp = KP;
    pid.Ki = KI;
    pid.Kd = KD;
    pid.tau = 0;
    pid.limMax = 8000.0F; //duty cycle 50% to limit max current drained to the heater
    pid.limMin = -8000.0F;
    pid.T = 0.5;//1; //    1 second
    pid_init(&pid);

}

float MeasureTemp(uint32_t temperature_adc)
{

    uint32_t Vo;
    uint8_t i=0;
    float R1 = 10000;
    float logR2, R2, tKelvin, tCelsius;
    float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;
    uint32_t pui32ADC0Value[2];//uint32_t pui32ADC0Value[4];
    uint16_t ReadingArray[filterSamples]; //the array for readings from the thermistor



    for (i = 0; i < filterSamples; i++)
    {
        // Trigger the ADC conversion.
        ADCProcessorTrigger(ADC0_BASE, 1); //3
        // Wait for conversion to be completed.
        while (!ADCIntStatus(ADC0_BASE, 1, false))//3
        {
        }
        // Clear the ADC interrupt flag.
        ADCIntClear(ADC0_BASE, 1);//3
        // Read ADC Value.
        //ADCSequenceDataGet(ADC0_BASE, 1, pui32ADC0Value);

        ADCSequenceDataGet(ADC0_BASE, 1, pui32ADC0Value);//3
        //UARTprintf("%4d\r\n", pui32ADC0Value[0]);
        ReadingArray[i] = pui32ADC0Value[0];   // Blue light data
        //delayMicroseconds(10);   // small delay between each reading
        SysCtlDelay(SysCtlClockGet() / 1200); //delay 0.025ms

    }


    Vo = pui32ADC0Value[0];
    Vo = SmoothData(ReadingArray,filterSamples);
    //UARTprintf("Vo = %4d\r\n", Vo);
    //UARTprintf("TIA = %4d\r\n",pui32ADC0Value[1]);

    R2 = R1 * (4095.0 / (float) Vo - 1.0); // resistance of the Thermistor
    logR2 = log(R2);
    tKelvin = (1.0 / (c1 + c2 * logR2 + c3 * logR2 * logR2 * logR2));
    tCelsius = tKelvin - 273.15;


    return tCelsius;

}
uint32_t measurePD(uint32_t temperature_adc)
{

    uint32_t Vo;
    uint8_t i=0;
    uint32_t pui32ADC1Value[2];//uint32_t pui32ADC0Value[4];
    uint16_t ReadingArray[filterSamples]; //the array for readings from the photodiode



    for (i = 0; i < filterSamples; i++)
    {
        // Trigger the ADC conversion.
        ADCProcessorTrigger(ADC1_BASE, 1); //3
        // Wait for conversion to be completed.
        while (!ADCIntStatus(ADC1_BASE, 1, false))//3
        {
        }
        // Clear the ADC interrupt flag.
        ADCIntClear(ADC1_BASE, 1);//3
        // Read ADC Value.
        //ADCSequenceDataGet(ADC0_BASE, 1, pui32ADC0Value);

        ADCSequenceDataGet(ADC1_BASE, 1, pui32ADC1Value);//3
        //UARTprintf("PD: %4d\r\n", pui32ADC1Value[0]);
        ReadingArray[i] = pui32ADC1Value[0];   // Blue light data
        //delayMicroseconds(10);   // small delay between each reading
        //SysCtlDelay(SysCtlClockGet() / 1200); //delay 0.025ms
        SysCtlDelay(SysCtlClockGet() / 1200);

    }


    Vo = pui32ADC1Value[0];
    Vo = SmoothData(ReadingArray,filterSamples);
    #ifdef DEBUG
        UARTprintf("ADC PD = %d\r\n", Vo);
    #endif



    return Vo;

}



//*****************************************************************************
//
// Configure the UART and its pins.  This must be called before UARTprintf().
//
//*****************************************************************************
void
ConfigureUART(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    MAP_GPIOPinConfigure(GPIO_PA0_U0RX);
    MAP_GPIOPinConfigure(GPIO_PA1_U0TX);
    MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);
}

void ConfigurePWM(void)
{
    //
    // Set the PWM clock to be equal to the system clock.
    //
    MAP_SysCtlPWMClockSet(SYSCTL_PWMDIV_1);

    //
    // The PWM peripheral must be enabled for use.
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);

    //
    // Enable the GPIO port that is used for the PWM output.
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    //
    // Configure the PWM function for this pin.
    //
    MAP_GPIOPinConfigure(GPIO_PB6_M0PWM0);
    MAP_GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6);

    //
    // Configure PWM0 to count down without synchronization.
    //
    MAP_PWMGenConfigure(PWM0_BASE, PWM_GEN_0,
                        PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);

    //
    // Set the PWM period to 250Hz.  To calculate the appropriate parameter
    // use the following equation: N = (1 / f) * SysClk.  Where N is the
    // function parameter, f is the desired frequency, and SysClk is the
    // system clock frequency.
    //
    //MAP_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, (SysCtlClockGet() / 250));
    MAP_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, (SysCtlClockGet() / 1000));
    //MAP_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, (SysCtlClockGet() / 100000));

    //
    // Set the PWM increment variable based on the System Clock. Since this
    // is a 250 Hz PWM, continue to use the equation N = (1 / f) * SysClk.
    // Then to set the initial period to 0.1% by dividing (N / 1000).
    // This variable will be used to increment PWM0 by 0.1% on each
    // interrupt.
    //
    //g_ui32PWMIncrement = ((SysCtlClockGet() / 250) / 1000);
    //g_ui32PWMIncrement = ((SysCtlClockGet() / 1000) / 100);              //160



    //
    // Set the initial PWM0 Pulse Width with the calculated increment variable
    // to start at 50% duty cycle.
    //
    //MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 6400);
    //MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0,MAP_PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0) * 1 / 4); //  2/4 gives 8000 as value ;  3/4 gives 12000 ; 4/4 16000
    //MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0,160); //  2/4 gives 8000 as value ;  3/4 gives 12000 ; 4/4 16000
    MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0,12000);  //0.1% duty cycle
    //MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0,MAP_PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0) * 3 / 4);

    //UARTprintf("Initial: %d",(int)MAP_PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0) * 3 / 4);

    //
    // Enable the PWM Out Bit 0 (PB6) output signal.
    //
    MAP_PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, true);

    //
    // Enable the PWM generator block.
    //
    MAP_PWMGenEnable(PWM0_BASE, PWM_GEN_0);



}
void ConfigureTimer(void){
    //
        // Enable the GPIO port that is used for the on-board LED.
        //
        MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

        //
        // Enable the GPIO pins for the LED (PF1 & PF2).
        //
        MAP_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_1);


        //
        // Enable the peripherals used by this example.
        //
        MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

        //
        // Enable processor interrupts.
        //
        MAP_IntMasterEnable();

        //
        // Configure the two 32-bit periodic timers.
        //
        MAP_TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);

        MAP_TimerLoadSet(TIMER0_BASE, TIMER_A, MAP_SysCtlClockGet() / 2); // *1 every 1 seconds

        //
        // Setup the interrupts for the timer timeouts.
        //
        MAP_IntEnable(INT_TIMER0A);

        MAP_TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);


        //
        // Enable the timers.
        //
        MAP_TimerEnable(TIMER0_BASE, TIMER_A);
}
void ConfigureTimerSensors(void){
    //
        //
        // Enable the peripherals used by this example.
        //
        MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);

        //
        // Enable processor interrupts.
        //
        MAP_IntMasterEnable();

        //
        // Configure the two 32-bit periodic timers.
        //
        MAP_TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);

        MAP_TimerLoadSet(TIMER1_BASE, TIMER_A, MAP_SysCtlClockGet() / 4 ); //every 4 seconds  change after

        //
        // Setup the interrupts for the timer timeouts.
        //
        MAP_IntEnable(INT_TIMER1A);

        MAP_TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);


        //
        // Enable the timers.
        //
        MAP_TimerEnable(TIMER1_BASE, TIMER_A);
}

void ConfigureADC(void){


         // The ADC0 peripheral must be enabled for use.
         //
         SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

         SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

         GPIOPinTypeADC(GPIO_PORTE_BASE,   GPIO_PIN_3 | GPIO_PIN_2  );
         //GPIOPinTypeADC(GPIO_PORTE_BASE,   GPIO_PIN_3   );
         //ADCReferenceSet(ADC0_BASE,ADC_REF_INT);

         ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);
         //ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);

         //ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END);
         ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH0 | ADC_CTL_IE );
         ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADC_CTL_CH1 | ADC_CTL_IE | ADC_CTL_END);

         ADCSequenceEnable(ADC0_BASE, 1);
         //ADCSequenceEnable(ADC0_BASE, 3);

         ADCIntClear(ADC0_BASE, 1);
         //ADCIntClear(ADC0_BASE, 3);

}
void ConfigureADC_PD(void){


         // The ADC0 peripheral must be enabled for use.
         //
        /* SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);

         SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

         GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_2  | GPIO_PIN_1 );

         ADCSequenceConfigure(ADC1_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);

         //ADCSequenceStepConfigure(ADC1_BASE, 3, 0, ADC_CTL_CH5 | ADC_CTL_IE | ADC_CTL_END);
         ADCSequenceStepConfigure(ADC1_BASE, 1, 0, ADC_CTL_CH5 | ADC_CTL_IE );
         ADCSequenceStepConfigure(ADC1_BASE, 1, 1, ADC_CTL_CH6 | ADC_CTL_IE | ADC_CTL_END);

         ADCSequenceEnable(ADC1_BASE, 1);
         //ADCSequenceEnable(ADC0_BASE, 3);

         ADCIntClear(ADC1_BASE, 1);
         //ADCIntClear(ADC0_BASE, 3);  */

             SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);

             SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

             GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_3  | GPIO_PIN_2 | GPIO_PIN_1  | GPIO_PIN_0 );

             ADCSequenceConfigure(ADC1_BASE, 1, ADC_TRIGGER_PROCESSOR, 0); //ui32SequenceNum = 1 means 4 samples
             /*void ADCSequenceConfigure(uint32_t ui32Base,uint32_t ui32SequenceNum,uint32_t ui32Trigger,uint32_t ui32Priority)*/

             ADCSequenceStepConfigure(ADC1_BASE, 1, 0, ADC_CTL_CH4 | ADC_CTL_IE );
             ADCSequenceStepConfigure(ADC1_BASE, 1, 1, ADC_CTL_CH5 | ADC_CTL_IE );
             ADCSequenceStepConfigure(ADC1_BASE, 1, 2, ADC_CTL_CH6 | ADC_CTL_IE );
             ADCSequenceStepConfigure(ADC1_BASE, 1, 3, ADC_CTL_CH7 | ADC_CTL_IE | ADC_CTL_END);


             ADCSequenceEnable(ADC1_BASE, 1);

             ADCIntClear(ADC1_BASE, 1);



}


//*****************************************************************************
//
// This example application demonstrates the use of the timers to generate
// periodic interrupts.
//
//*****************************************************************************
int
main(void)
{
    //
    // Enable lazy stacking for interrupt handlers.  This allows floating-point
    // instructions to be used within interrupt handlers, but at the expense of
    // extra stack usage.
    //
    MAP_FPULazyStackingEnable();
    FPUEnable();

    //
    // Set the clocking to run directly from the crystal.
    //
    MAP_SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |
                       SYSCTL_XTAL_16MHZ);

    //
    // Initialize the UART and write status.
    //
    ConfigureUART();

    ConfigurePWM();

    ConfigureTimer();

    //ConfigureTimerSensors();

    ConfigureADC();

    ConfigureADC_PD();

    initPID();



    //
    // Loop forever while the timers run.
    //
    while(1)
    {
    }
}

void setPWM(int32_t heater_PWM)
{

    int32_t PWM_2b_set=0;

    PWM_2b_set = (int32_t) (MAP_PWMPulseWidthGet(PWM0_BASE, PWM_OUT_0)+ heater_PWM);
    #ifdef DEBUG
    UARTprintf("PWM value: %d\n",
               (int32_t) MAP_PWMPulseWidthGet(PWM0_BASE, PWM_OUT_0));
    UARTprintf("PWM value TO BE SET 2 : %d\n",PWM_2b_set);
    #endif




    if( PWM_2b_set <= (int32_t)((MAP_PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0) * 3) / 4))

    {
         //if((heater_PWM_lastone>=(heater_PWM+5)) && (heater_PWM>0) ){
        if(PWM_2b_set < 0){
            //UARTprintf("PWM less than ZERO!!!!\n\n");
            MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0,160);
        }
        else{
            if((heater_PWM<0)){
             //MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0,MAP_PWMPulseWidthGet(PWM0_BASE, PWM_GEN_0) - (heater_PWM_lastone-heater_PWM));
                MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0,MAP_PWMPulseWidthGet(PWM0_BASE, PWM_GEN_0) + heater_PWM);
                #ifdef DEBUG
                     UARTprintf("DIMINIU!!!\n\n");
                #endif
            }else{
                MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0,MAP_PWMPulseWidthGet(PWM0_BASE, PWM_GEN_0) + heater_PWM);
                #ifdef DEBUG
                      UARTprintf("AUMENTOU!!!\n\n");
                 #endif
            }
        }
    }
    else
    {
     //PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, g_ui32PWMIncrement);
     //MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0,MAP_PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0) * 4 / 4);
       MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0,12000);
       #ifdef DEBUG
         UARTprintf("ESTOUROU\n\n");
       #endif

    }

     heater_PWM_lastone = heater_PWM;

}

//*****************************************************************************
//
// The interrupt handler for the first timer interrupt.
//
//*****************************************************************************
void Timer0IntHandler(void)
{
    //char cOne, cTwo;
    float temperature=0;


    //
    // Clear the timer interrupt.
    //
    MAP_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    //
    // Toggle the flag for the first timer.
    //
    HWREGBITW(&g_ui32Flags, 0) ^= 1;

    //
    // Use the flags to Toggle the LED for this timer
    //
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, g_ui32Flags << 1);


    //
    // Update the interrupt status on the display.
    //
    MAP_IntMasterDisable();


    temperature = MeasureTemp(1);

    UARTprintf("%d,",(int32_t) MAP_PWMPulseWidthGet(PWM0_BASE, PWM_OUT_0));
    UARTprintf("%d,", (int)temperature);

    measureADC();



    #ifdef ONLY_ADC
        UARTprintf("2000,");
        UARTprintf("%d,", (int)temperature);
        UARTprintf("%d\n",measurePD(2));
        //UARTprintf("Temperature (celsius): %d\r\n", (int)temperature);
    #endif



    #ifdef DEBUG
        UARTprintf("Temperature (celsius): %d\r\n", (int)temperature);
    #endif

    #ifdef PLOT_TEMP
        UARTprintf("%d,",(int32_t) MAP_PWMPulseWidthGet(PWM0_BASE, PWM_OUT_0));
        UARTprintf("%d,", (int)temperature);
        UARTprintf("%d\n",measurePD(2));
    #endif

    #ifdef SENSOR
        if(goPD==4){
            UARTprintf("%d,",(int32_t) MAP_PWMPulseWidthGet(PWM0_BASE, PWM_OUT_0));
            UARTprintf("%d,", (int)temperature);
            UARTprintf("%d\n",measurePD(2));
            //measurePD(2);
            goPD=0;
            //UARTprintf("*************************\n");
        }else{
            goPD+=1;
        }

    #endif



    if ((setPoint <= (uint16_t) temperature) || (flagPID==1))
    {
        #ifdef DEBUG
            UARTprintf("Starting PI Controller...\r\n");
        #endif


            flagPID=1;

            pid_update(&pid, setPoint,temperature);
            //UARTprintf("PID out: %d\r\n", (int)pid.out);

            g_ui32PWMIncrement = (int32_t)pid.out;
            //UARTprintf("PID out (int32): %d\r\n", g_ui32PWMIncrement);

            if(PIDfirst_time==0){
                MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0,5000);//MAP_PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0) * 1 / 4); //* 5 / 16
                PIDfirst_time=1;
                #ifdef DEBUG
                    UARTprintf("Setting PWM to 25%...\r\n");
                #endif
            }

            setPWM(g_ui32PWMIncrement);

    }


    MAP_IntMasterEnable();
}

//*****************************************************************************
//
// The interrupt handler for the second timer interrupt.
//
//*****************************************************************************
void Timer1IntHandler(void)
{

    //
    // Clear the timer interrupt.
    //
    MAP_TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

    //
    // Update the interrupt status on the display.
    //
    MAP_IntMasterDisable();
    UARTprintf("Timer Sensors...\n");
    MAP_IntMasterEnable();
}





