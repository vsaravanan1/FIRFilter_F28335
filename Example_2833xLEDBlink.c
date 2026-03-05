//
// Included Files
//
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "fpu_types.h"   // MUST come before fpu_filter.h

#include "DSP28x_Project.h"
#include "fpu_filter.h"
#include "fir_filter.h"  // Your data file

#define BUF_SIZE 100
#define FIR_OUTPUT_BUF_SIZE 512

//macros for max and min
#define max(a, b) (((a > b) ? (a) : (b)))
#define min(a, b) (((a < b) ? (a) : (b)))

//epwm3
double rms_double = 1.0;
uint16_t cca_value = 50;


// dma and adc pointers (arrays)
volatile uint16_t* DMADestCh1 = NULL;
volatile uint16_t* DMASource = NULL;
volatile uint16_t* DMADestCh2 = NULL;
volatile uint16_t* DMADestCh3 = NULL;
uint16_t ADC_BUF_1[BUF_SIZE] = {};
uint16_t ADC_BUF_2[BUF_SIZE] = {};

//fir
uint16_t fir_output_buf_idx = 0;
float fir_output_buf[FIR_OUTPUT_BUF_SIZE];
float alpha = 0.0002;
float mean_sq = 0.0;
float V_sq = 0;


//RMS value
uint64_t NUM_SAMPLES = 1;

//
// Function Prototypes
//
__interrupt void cpu_timer0_isr(void);
__interrupt void dintch1_isr(void);
__interrupt void dintch2_isr(void);
__interrupt void epwm2_isr(void);



void init_epwm(void);
void sci_init(void);
void configure_fir(void);
void configure_dma(void);
void filter_data(int buf_num);


unsigned short ch1_write = 0;
unsigned short ch2_write = 0;

FIR_f32 fir_filter = FIR_f32_DEFAULTS;
FIR_f32_Handle fir_handle = &fir_filter;



//
// Main
//
void main(void)
{

    InitSysCtrl();
    InitGpio();


    // Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    DINT;

    //
    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the DSP2833x_PieCtrl.c file.
    //
    InitPieCtrl();

    //
    // Disable CPU interrupts and clear all CPU interrupt flags:
    //
    IER = 0x0000;
    IFR = 0x0000;

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // The shell ISR routines are found in DSP2833x_DefaultIsr.c.

    InitPieVectTable();

    //
    // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this file.
    //
    EALLOW;         // This is needed to write to EALLOW protected registers
    PieVectTable.TINT0 = &cpu_timer0_isr;
    PieVectTable.DINTCH1 = &dintch1_isr;
    PieVectTable.DINTCH2 = &dintch2_isr;
    PieVectTable.EPWM2_INT = &epwm2_isr;
    EDIS;    // This is needed to disable write to EALLOW protected registers


    //
    // Step 4. Initialize the Device Peripheral. This function can be
    //         found in DSP2833x_CpuTimers.c
    //
    InitCpuTimers();   // For this example, only initialize the Cpu Timers
#if (CPU_FRQ_150MHZ)
    //
    // Configure CPU-Timer 0 to interrupt every 500 milliseconds:
    // 150MHz CPU Freq, 50 millisecond Period (in uSeconds)
    //
    ConfigCpuTimer(&CpuTimer0, 150, 500000);
#endif
#if (CPU_FRQ_100MHZ)
    //
    // Configure CPU-Timer 0 to interrupt every 500 milliseconds:
    // 100MHz CPU Freq, 50 millisecond Period (in uSeconds)
    //
    ConfigCpuTimer(&CpuTimer0, 100, 500000);
#endif


#if (CPU_FRQ_150MHZ)     // Default - 150 MHz SYSCLKOUT
    //
    // HSPCLK = SYSCLKOUT/2*ADC_MODCLK2 = 150/(2*3)   = 25.0 MHz
    //
    #define ADC_MODCLK 0x3
#endif
#if (CPU_FRQ_100MHZ)
    //
    // HSPCLK = SYSCLKOUT/2*ADC_MODCLK2 = 100/(2*2)   = 25.0 MHz
    //
    #define ADC_MODCLK 0x2
#endif

    //
    // To ensure precise timing, use write-only instructions to write to the
    // entire register. Therefore, if any of the configuration bits are changed
    // in ConfigCpuTimer and InitCpuTimers (in DSP2833x_CpuTimers.h), the
    // below settings must also be updated.
    //

    //
    // Use write-only instruction to set TSS bit = 0
    //
    CpuTimer0Regs.TCR.all = 0x4000;

    //
    // Step 5. User specific code, enable interrupts
    //
#if (_EZDSP_BOARD)
    //
    // Configure GPIO32 as a GPIO output pin
    //
    EALLOW;
    GpioCtrlRegs.GPBMUX1.bit.GPIO32 = 0;
    GpioCtrlRegs.GPBDIR.bit.GPIO32 = 1;
    EDIS;
#else
    //
    // Configure GPIO31 as a GPIO output pin
    //
    EALLOW;
    GpioCtrlRegs.GPAMUX2.bit.GPIO31 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO31 = 1;
    EDIS;
#endif

    // HSPCLK = SYSCLKOUT/ADC_MODCLK (ADC peripheral clock)
    EALLOW;
       SysCtrlRegs.HISPCP.all = ADC_MODCLK;
    EDIS;

    // Enable the SCI-A peripheral clock
    EALLOW;
       SysCtrlRegs.PCLKCR0.bit.SCIAENCLK = 1;
    EDIS;

    // power up adc
    InitAdc();

    // configure ADC
    AdcRegs.ADCTRL2.bit.RST_SEQ1 = 1;
    AdcRegs.ADCTRL1.bit.ACQ_PS = 0x00;
    AdcRegs.ADCTRL3.bit.ADCCLKPS = 0x00;
    AdcRegs.ADCTRL1.bit.SEQ_CASC = 0;
    AdcRegs.ADCTRL2.bit.EPWM_SOCA_SEQ1 = 1;
    AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1 = 1;
    AdcRegs.ADCTRL2.bit.INT_MOD_SEQ1 = 0;
    AdcRegs.ADCCHSELSEQ1.bit.CONV00 = 0;
    AdcRegs.ADCMAXCONV.bit.MAX_CONV1 = 0;

    init_epwm();
    configure_fir();
    configure_dma();

    int j;
    for (j = 0; j < BUF_SIZE; j++) {
        ADC_BUF_1[j] = 0;
        ADC_BUF_2[j] = 0;
    }


    //
    // Enable CPU INT1 which is connected to CPU-Timer 0:
    //
    IER |= M_INT1;
    IER |= M_INT7;
    IER |= M_INT3;

    //
    // Enable TINT0 in the PIE: Group 1 interrupt 7
    //
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    PieCtrlRegs.PIEIER7.bit.INTx1 = 1;
    PieCtrlRegs.PIEIER7.bit.INTx2 = 1;
    PieCtrlRegs.PIEIER3.bit.INTx2 = 1;


    //
    // Enable global Interrupts and higher priority real-time debug events
    //
    EINT;           // Enable Global interrupt INTM
    ERTM;           // Enable Global realtime interrupt DBGM


    //
    // Loop (does fir filter processing)
    //
    for(;;) {
        if (ch1_write == 1) {
            SciaRegs.SCITXBUF = 'a';
            while(!(SciaRegs.SCICTL2.bit.TXRDY));
            SciaRegs.SCITXBUF = '\n';
            while(!(SciaRegs.SCICTL2.bit.TXRDY));
            filter_data(1);
            ch1_write = 0;
        }

        if (ch2_write == 1) {
            SciaRegs.SCITXBUF = 'b';
            while(!(SciaRegs.SCICTL2.bit.TXRDY));
            SciaRegs.SCITXBUF = '\n';
            while(!(SciaRegs.SCICTL2.bit.TXRDY));
            filter_data(2);
            ch2_write = 0;
        }

    }

    EALLOW;
        SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    EDIS;
}

//
// cpu_timer0_isr -
//

void init_epwm(void) {
    //gpio config for EPWM3
    EALLOW;
        GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;
        GpioCtrlRegs.GPAPUD.bit.GPIO4 = 1;
    EDIS;


    EALLOW;
    EPwm1Regs.TBPRD = 29;
    EPwm1Regs.TBCTL.bit.CTRMODE = 0;
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = 0;
    EPwm1Regs.TBCTL.bit.CLKDIV = 0;

    EPwm1Regs.ETSEL.bit.SOCAEN = 1;
    EPwm1Regs.ETSEL.bit.SOCASEL = 0b010;
    EPwm1Regs.ETPS.bit.SOCAPRD = 1;

    EPwm2Regs.TBPRD = 11719;
    EPwm2Regs.TBCTL.bit.CTRMODE = 0;
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = 0b101;
    EPwm2Regs.TBCTL.bit.CLKDIV = 0b100;
    EPwm2Regs.ETSEL.bit.INTEN = 1;
    EPwm2Regs.ETSEL.bit.INTSEL = 0b010;
    EPwm2Regs.ETPS.bit.INTPRD = 0b01;


    EPwm3Regs.TBPRD = 30000;
    EPwm3Regs.TBCTL.bit.CTRMODE = 0;
    EPwm3Regs.TBCTL.bit.HSPCLKDIV = 0b101;
    EPwm3Regs.TBCTL.bit.CLKDIV = 0b010;
    EPwm3Regs.CMPA.half.CMPA = 15000;
    EPwm3Regs.AQCTLA.bit.CAU = 0b10;
    EPwm3Regs.AQCTLA.bit.ZRO = 0b01;

    EDIS;
}

void configure_fir(void) {
    FIR_f32_setCoefficientsPtr(fir_handle, coeffs);
    FIR_f32_setDelayLinePtr(fir_handle, delay_line_buffer);
    FIR_f32_setCalcFunction(fir_handle, FIR_f32_calc);
    FIR_f32_setInitFunction(fir_handle, FIR_f32_init);
    FIR_f32_setOrder(fir_handle, order);
    FIR_f32_setInput(fir_handle, input);
    FIR_f32_setOutput(fir_handle, output);
    FIR_f32_init(fir_handle);
}

void sci_init(void) {
    EALLOW;
        GpioCtrlRegs.GPAPUD.bit.GPIO28 = 0;
        GpioCtrlRegs.GPAPUD.bit.GPIO29 = 0;
        GpioCtrlRegs.GPAQSEL2.bit.GPIO28 = 3;
        GpioCtrlRegs.GPAQSEL2.bit.GPIO29 = 3;
        GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 1;
        GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 1;
    EDIS;

    SciaRegs.SCICTL1.bit.SWRESET = 0;
    SciaRegs.SCICCR.bit.SCICHAR = 7;
    SciaRegs.SCICTL1.bit.RXENA = 1;
    SciaRegs.SCICTL1.bit.TXENA = 1;

    // baud rate of 937500, assuming LSPCLK = 37.5M
    SciaRegs.SCIHBAUD = 0x0000;
    SciaRegs.SCILBAUD = 0x0004;
    SciaRegs.SCICTL1.bit.SWRESET = 1;

}


void configure_dma(void) {
    // DMA channels 1 and 2 config (ping-pong ADC buffer, start channel 1 initially, then start ch2 in isr once ch1 ends)
    DMADestCh1 = &ADC_BUF_1[0];
    DMASource = &AdcMirror.ADCRESULT0;
    DMACH1AddrConfig(DMADestCh1, DMASource);
    DMACH1BurstConfig(0,0,0);
    DMACH1TransferConfig(BUF_SIZE-1,0,1);
    DMACH1WrapConfig(0,0,BUF_SIZE-1,0);
    DMACH1ModeConfig(DMA_SEQ1INT,PERINT_ENABLE,ONESHOT_DISABLE,CONT_DISABLE,SYNC_DISABLE,SYNC_SRC,
                        OVRFLOW_DISABLE,SIXTEEN_BIT,CHINT_END,CHINT_ENABLE);
    StartDMACH1();

    DMADestCh2 = &ADC_BUF_2[0];
    DMASource = &AdcMirror.ADCRESULT0;
    DMACH2AddrConfig(DMADestCh2, DMASource);
    DMACH2BurstConfig(0,0,0);
    DMACH2TransferConfig(BUF_SIZE-1,0,1);
    DMACH2WrapConfig(0,0,BUF_SIZE-1,0);
    DMACH2ModeConfig(DMA_SEQ1INT,PERINT_ENABLE,ONESHOT_DISABLE,CONT_DISABLE,SYNC_DISABLE,SYNC_SRC,
                           OVRFLOW_DISABLE,SIXTEEN_BIT,CHINT_END,CHINT_ENABLE);


}

void filter_data(int buf_num) {
    if (fir_output_buf_idx == FIR_OUTPUT_BUF_SIZE) {
        fir_output_buf_idx = 0;
    }

    if (buf_num == 1) {
        for (int i = 0; i < BUF_SIZE; i++) {
            fir_handle->input = (float)ADC_BUF_1[i];
            fir_handle->calc(fir_handle);
            fir_output_buf[fir_output_buf_idx] = fir_handle->output;
            fir_output_buf_idx++;
            V_sq = (fir_handle->output) * (fir_handle->output);
            mean_sq = alpha * (V_sq) + (1 - alpha) * (mean_sq);
        }
    }

    else if (buf_num == 2) {
        for (int i = 0; i < BUF_SIZE; i++) {
            fir_handle->input = ADC_BUF_2[i];
            fir_handle->calc(fir_handle);
            fir_output_buf[fir_output_buf_idx] = fir_handle->output;
            fir_output_buf_idx++;
            V_sq = (fir_handle->output) * (fir_handle->output);
            mean_sq = alpha * (V_sq) + (1 - alpha) * (mean_sq);
        }
    }
}


__interrupt void
dintch1_isr(void)
{
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP7;
   ch1_write = 1;

   StartDMACH2();
}


__interrupt void
dintch2_isr(void)
{
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP7;
   ch2_write = 1;
   StartDMACH1();
}


__interrupt void
epwm2_isr(void)
{
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
    rms_double = sqrt(mean_sq);
    double duty_ratio = rms_double/(4096.0);
    duty_ratio = min(max(duty_ratio, 0.0), 1.0);
    cca_value = (uint16_t)(30000.0 * (1 - duty_ratio));
    cca_value = min(max(cca_value, 0), 30000);

    EALLOW;
    EPwm3Regs.CMPA.half.CMPA = cca_value;
    EDIS;

    EPwm2Regs.ETCLR.bit.INT = 1;
}


__interrupt void
cpu_timer0_isr(void)
{
    CpuTimer0.InterruptCount++;

#if (_EZDSP_BOARD)
    //
    // Toggle GPIO32 once per 500 milliseconds
    //
    GpioDataRegs.GPBTOGGLE.bit.GPIO32 = 1;
#else
    //
    // Toggle GPIO31 once per 500 milliseconds
    //
    GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;
#endif

    //
    // Acknowledge this interrupt to receive more interrupts from group 1
    //
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

//
// End of File
//

