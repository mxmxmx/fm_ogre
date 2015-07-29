/*
 * File:   FMOgre_DMA_main.c for FMOgre_44 (44-pin surface-mount) board
 * Author: wsy
 *
 * Created on August 16, 2013, 7:10 AM
 *    This file is Copyright 2013 William S. Yerazunis,
 *   It is licensed CC-BY-SA-NC
 *
 *   (creative commons, attribute source, share alike, no commercial
 *    use without other license)
 *
 *  adapted for mxmxmx layout; basically, this boils down to:
 *  RB5 is unused (was: heartbeat/LED), as is RA8 (was: switch #1); the ADC is mapped as follows:
 *
 * cvpitch :: RB1
 * cvfb :: RA1
 * cvfm :: RB0
 * cvpm :: RA0
 * cvpmknob :: RC0
 * cvfmknob :: RB3 
 * cvfbknob :: RC1
 * cvpitchknob :: RB2
 *
 * heartbeat/LED top :: RB6
 * neg. freq./LED bottom :: RB7
 * switch (top) :: RA4
 * switch (bottom) :: RB4
 */

/*
 *    Steps in the process:
 *
 *  - Initialize the IO pins:
 *     - analog outputs:
 *          10(DAC1RP : primary output), 11(DAC1RN), 14(DAC1LP), 15(DAC1LN) (RB12,13,14,15)
 *     - analog inputs:
 *          AN0 (19, RA0):  exponential pitch jack
 *          AN1 (20, RA1):  operator feedback jack
 *          AN2 (21, RB0):  freq mod jack
 *          AN3 (22, RB1):  phase mod jack
 *          AN4 (23, RB2):  phase mod index knob
 *          AN5 (24, RB3):  fm mod index knob
 *          AN6 (25, RC0):  operator feedback knob
 *          AN7 (26, RC1):  pitch knob
 *     - digital inputs:
 *          RA2 (30):   (also crystal in)
 *          RA3 (31):   (also crystal out)  ???  if low, use RA2 as 7.37MHz osc in ???
 *          RB4 (33):
 *          RA4 (34):
 *          pin 17,28,40: Vdd (+3.3v)
 *          RB5 (41): / PGED3 (5Vtol)  unused
 *          RB6 (42): / PGEC3 (5Vtol)  idle hearbeat
 *          RB7 (43):         (5Vtol)  negative phase
 *          RB8 (44):         (5Vtol)  Sync Out (25% duty cycle pulse train)
 *          RB9  (1):         (5Vtol)  Sync In
 *          pins 6, 16, 29, 39: Vss (ground)
 *          pin 7: Vcap (core voltage stabilizer)
 *          pin 21: PGED2 / RB10 (5Vtol) - used for programming
 *          pin 22: PGEC2 / RB11 (5Vtol) - used for programming
 *          23-26 = RP, RN, LP, and LN analog out = see above
 *          pin 27: AVss
 *          pin 28: AVdd
 *
*/

//    Data structure(s)
//
//    fmphase, outphase: 32 bit unsigned ints.   Both are used in the
//    positive-only range, and are abused as follows:
//    2^31 to 2^19 - used for output (12 bits into the LUT, same as the DX7 had).
//    2^18 to 2^0 - fractional phase, not used but carried over which maintains
//    phase coherency
//
//      By using this format, it is unnecessary to rectify phase wrapping in
//      either directions - overflow and underflow can be IGNORED.The latter also
//      makes this do thru-zero FM work correctly.

//    Pseudocode / Design (May be out of date):
//    - initialize on-board RC clock (and crystal clock and dividers)
//    - initialize ADC (unsigned 12 bit)
//    - initialize DAC (unsigned 16 bit 48 KHz)
//    - initialize shared locations
//    - set up ADC interrupt service
//    - set up DAC interrupt service
//    - while (1)
//      - outphase = fmphase + (phasemodval * phasemodindex)
//      - clip outphase into 0 to outphasemax-1
//      - read RB8 and RB9 (to choose LUT)
//      - prepend RB9:RB8 to get lutindex
//      -
//    * ADC interrupt service:
//      - read the ADC register
//      - put result in the appropriate ADCoutput buffer
//      - kick off another conversion (AN0, 1, 2, 3, 4, 5, 0, 1, 2...)
//      - fmphaseincr = (freqmodval * freqmodindex) + (pitchval * pitchscale)  ***OWNED TRANSITION
//      - fmphase = fmphase + fmphaseincr
//      - clip fmphase to 0 to fmphasemax-1
//      - return
//


#include <stdio.h>
/// #include <pic16f1825.h>
///#include <dspic33fj64gp802.h>
#include <p33FJ128GP804.h>
//#include "common.h"
#include "dsp.h"

// DSPIC33FJ64GP802 Configuration Bit Settings

#include <p33Fxxxx.h>

// FBS
#pragma config BWRP = WRPROTECT_OFF     // Boot Segment Write Protect (Boot Segment may be written)
#pragma config BSS = NO_FLASH           // Boot Segment Program Flash Code Protection (No Boot program Flash segment)
#pragma config RBS = NO_RAM             // Boot Segment RAM Protection (No Boot RAM)

// FSS
#pragma config SWRP = WRPROTECT_OFF     // Secure Segment Program Write Protect (Secure segment may be written)
#pragma config SSS = NO_FLASH           // Secure Segment Program Flash Code Protection (No Secure Segment)
#pragma config RSS = NO_RAM             // Secure Segment Data RAM Protection (No Secure RAM)

// FGS
#pragma config GWRP = OFF               // General Code Segment Write Protect (User program memory is not write-protected)
#pragma config GSS = OFF                // General Segment Code Protection (User program memory is not code-protected)

// FOSCSEL
//   Uncomment next for FRC PLL
//#pragma config FNOSC = FRCPLL           // Oscillator Mode (Internal Fast RC (FRC) w/ PLL)
//   Uncomment next for crystal
//#pragma config FNOSC = PRI              // Oscillator Mode (Primary Oscillator (XT, HS, EC))
//#pragma config IESO = ON                // Internal External Switch Over Mode (Start-up device with FRC, then automatically switch to user-selected oscillator source when ready)

// FOSC

//  Uncomment next four for RC high speed clock
//#pragma config POSCMD = HS              // Primary Oscillator Source (HS Oscillator Mode)
//#pragma config OSCIOFNC = ON            // OSC2 Pin Function (OSC2 pin has digital I/O function)
//#pragma config IOL1WAY = OFF            // Peripheral Pin Select Configuration (Allow Multiple Re-configurations)
//#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor (Both Clock Switching and Fail-Safe Clock Monitor are disabled)

//   Uncomment next four for crystal oscillator mode.
//#pragma config POSCMD = XT              // Primary Oscillator Source (XT Oscillator Mode)
//#pragma config OSCIOFNC = OFF           // OSC2 Pin Function (OSC2 pin has clock out function)
//#pragma config IOL1WAY = OFF            // Peripheral Pin Select Configuration (Allow Multiple Re-configurations)
//#pragma config FCKSM = CSECMD           // Clock Switching and Monitor (Clock switching is enabled, Fail-Safe Clock Monitor is disabled)
_FOSCSEL (FNOSC_PRIPLL & IESO_ON )
_FOSC (POSCMD_XT & OSCIOFNC_OFF & IOL1WAY_OFF & FCKSM_CSECME)

// FWDT
#pragma config WDTPOST = PS32768        // Watchdog Timer Postscaler (1:32,768)
#pragma config WDTPRE = PR128           // WDT Prescaler (1:128)
#pragma config WINDIS = OFF             // Watchdog Timer Window (Watchdog Timer in Non-Window mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (Watchdog timer enabled/disabled by user software)

// FPOR
#pragma config FPWRT = PWR16            // POR Timer Value (16ms)
#pragma config ALTI2C = OFF             // Alternate I2C  pins (I2C mapped to SDA1/SCL1 pins)

// FICD
#pragma config ICS = PGD2               // Comm Channel Select (Communicate on PGC2/EMUC2 and PGD2/EMUD2)
#pragma config JTAGEN = OFF             // JTAG Port Enable (JTAG is Disabled)


volatile unsigned long iz;
volatile unsigned long zig;
volatile unsigned long zurg;
//     FM state variables
volatile unsigned long curbasephase;
volatile long curpitchval;
volatile long curpitchincr;
volatile long curfreqmod;
volatile long curphasemod;
//     AD converter stuff
int which_adc;
unsigned long sinevalue;
volatile unsigned long final_phase_fm;
volatile unsigned long final_phase_fmpm;
volatile unsigned long final_phase_fm_feedback;
volatile long phase_shift;
volatile long phaselowpass;
volatile char oldhardsync;
volatile char _PHASEMOD;
volatile char prevStateRA4;

//     to make stuffing the ADC values easier, we use some DEFINEs:
//     changed to reflect pins used (mxmxmx))
volatile unsigned long cvdata[12];
#define cvpitch (cvdata[3]) // RB1
#define cvfb (cvdata[1]) // RA1
#define cvfm (cvdata[2]) // RB0
#define cvpm (cvdata[0]) // RA0
#define cvpmknob (cvdata[6]) // RC0
#define cvfmknob (cvdata[5]) // RB3 
#define cvfbknob (cvdata[7]) // RC1
#define cvpitchknob (cvdata[4]) // RB2
#define ADC_FIRST (0)
#define ADC_LAST (7)

static short adc_inverted [8] = {1,1,1,1,0,0,0,0};
//static short adc_sequence [16] = {0,0,1,1,2,2,3,3,4,4,5,5,6,6,7,7};

//   Pins RB6, and RB7 are the LED port.
//   Pin  RB8 is SYNC OUT, pin RB9 is SYNC IN.
//     The wavetable for sine waves are in wavetable.h
#include "wavetable.h"

void __attribute__((__interrupt__,__auto_psv__)) _ADC1Interrupt(void)
{
    PORTCbits.RC6 = 1;    // RC1 used as a telltale "In AD Conversion"
    //     Set up the ADC for the next sample
    AD1CON1bits.SAMP = 1;           //  start the sampling time for the ADC
    //                              //   Small race condition here - we
    //                              // grab the ADC data and stuff it into memory
    //
//#define PRECHARGE_ADC
#ifdef PRECHARGE_ADC
    static short chargeonly ;
    if (chargeonly )
        {
            chargeonly--;
        }
    else
    {
        //chargeonly = 0;            // use chargeonly if your input drivers are weak
        if (adc_inverted[which_adc])
            cvdata [which_adc] =  ((unsigned long)4096) - ADC1BUF0;
        else
            cvdata [which_adc] = ADC1BUF0;
        which_adc++;
    }
#else
    if (adc_inverted[which_adc])
        cvdata [which_adc] =  ((unsigned long)4096) - ADC1BUF0;
    else
        cvdata [which_adc] = ADC1BUF0;
    which_adc++;
#endif
    //  NEXT LINE:  when only unipolar inverted inputs are available, use next line.
    //   ADC1BUF is in range 2048-4096
    //cvdata[which_adc] = (4096 - ADC1BUF0) * 2;

    //      The ADC has loaded the S/H now, so we can now move on to the next channel
    if (which_adc > ADC_LAST) which_adc = ADC_FIRST;
    AD1CHS0bits.CH0SA = which_adc;     //  channel A positive input is input which_adc
    AD1CON1bits.DONE = 0;             //  clear DONE status
    IFS0bits.AD1IF = 0;               //   clear the interupt
    AD1CON1bits.SAMP = 0;              //  kick the ADC into conversion
    //   From this instant forward, we have about half a microsecond to do everything
    //   else before the DAC finishes and we have to service this interrupt again.

    //    The following is for seeing just how often this interrupt gets triggered.
    //    iz++;
    //    PORTBbits.RB6 = 0x1 & iz;
    //    PORTBbits.RB6 = ! (PORTBbits.RB6);
    PORTCbits.RC6 = 0;

}

void __attribute__((__interrupt__,__auto_psv__)) _DAC1LInterrupt(void)
{
    //IFS4bits.DAC1LIF = 0;             //   clear the interupt
    //IFS4bits.DAC1RIF = 1;             //   fire the right channel interrupt
    //DAC1LDAT = sine_table [0x0000FFF & ( curbasephase >> 20)];
}

void __attribute__((__interrupt__,__auto_psv__)) _DAC1RInterrupt(void)
{
    //    The analog outputs are all driven from this interrupt.
    //    Whenever the DAC finishes and creates space in the FIFO, this
    //    interrupt fires off to fill that space in the FIFO.   Since
    //    the DAC itself is clocked by a PLL from the crystal, the result
    //    is a very stable tone generation IFF the input freq. and phase
    //    are solid and unchanging (they aren't- the ADCs are a bit noisy)

    IFS4bits.DAC1RIF = 0;             //    clear the interrupt

    //    CVPITCHINCR now gets calculated in the mainline ?  used to do it here
    //    but it ate too much CPU and slowed down the ADC interrupt.
//#define INTERRUPT_CURPITCHINCR
#ifdef INTERRUPT_CURPITCHINCR
    curpitchval = cvpitchknob + ( ( cvpitch) - 2048) ;
    //   use exp_table to get exp freq resp.
    curpitchincr = exp_table [
            (curpitchval < 0) ? 0 :
                (curpitchval > 0x0FFF) ? 0x0FFF : curpitchval];
#endif
    //        MIDI and Frequency to curfreqincr conversion NOTES:
    //        MIDI note 0 = 8.1757 Hz.  MIDI note 127 = 12543 Hz
    //     use 23616894 for 83333 sample rate and 440 Hz out = MIDI note 69 +/- 0.1%
    // UNCOMMENT NEXT LINE FOR A-440 frequency output
    //curpitchincr = 23599607 ;

    curbasephase = curbasephase + curpitchincr;


    //   Calculate the linear frequency modulation.  Used to do it here,
    //   now do it in the mainline to speed up the interrupt.
//#define INTERRUPT_CURFREQMOD
#ifdef INTERRUPT_CURFREQMOD
    curfreqmod = cvfmknob * (cvfm- 2048) << 5;
    //    Output RB6 high if we have negative frequency
    TRISBbits.TRISB6 = curpitchincr + curfreqmod < 0 ? 0 : 1;
#endif

    //   Add in linear frequency to the pitch base phase.
    curbasephase = curbasephase + curfreqmod;


//#define RAW_PLUS_LINEAR_FM_CHECK
#ifdef RAW_PLUS_LINEAR_FM_CHECK
    DAC1RDAT = sine_table [0x00000FFF & (curbasephase >> 20)];
    DAC1LDAT = sine_table [0x00000FFF & (curbasephase >> 20)];
    return;
#endif
    //DAC1DFLT = sine_table [0x0000FFF & ( curbasephase >> 20)];

    //   Sinevalue is the output of the basephase->sine conversion BUT it's
    //   not used directly.  Instead, it's the intermediate for the operator
    //   feedback, and used to generate the final (actual) FM phase.
    //   Because it's synchronous with curbasephase, we *do* calculate it at
    //   interrupt time (fortunately, it's fast)
    sinevalue = sine_table [0x0000FFF & (curbasephase >> 20)] ;

    //   Used to calculate curphasemod here, but now do it in mainline (again, to
    //   minimize time spent in the interrupt and thus not slow down the ADC stream)
//#define INTERRUPT_CURPHASEMOD
#ifdef INTERRUPT_CURPHASEMOD
    curphasemod = (((cvpm - 2048) * cvpmknob) >> 12);
                //+ ((curphasemod ) / 2);
    //   Output RB7 driven high if we have negative phase
    TRISBbits.TRISB7 = curpitchincr + (curphasemod << 13) < 0 ? 0 : 1 ;
#endif

#ifdef WORKING_VERSION
    final_phase_fm = 0x0000FFF &
                    (
                        (                            // native base phase
                            ( curbasephase >> 20)
                                +                             //  plus operator feedback
                            ( (sinevalue - 32767) * ((cvfbknob * cvfb)>> 12) >> 12)
                            +
                            ( curphasemod  )
                        )
                    );

    //   All done - stuff the DAC output registers.
    DAC1RDAT = sine_table [0x00000FFF & ((curbasephase >> 20) + 2048)];
    DAC1LDAT = sine_table [0X00000fff & (final_phase_fm + 2048)];
    return;
#endif
    //
    //   Slightly experimental version.   FM + Feedback on one, FM+PM+FB on other
    final_phase_fm_feedback = 0x00000FFF &
                    (
                        (
                            (curbasephase >> 20)    //  native base phase
                                +                       // plus operator feedback
                            ( (sinevalue - 32767 ) * ((cvfbknob * cvfb) >> 12) >> 12)
                         )
                    );
    final_phase_fm = 0x00000FFF &
                    (
                        (                            // native base phase
                            ( curbasephase >> 20)
                                +                             //  plus operator feedback
                            ( (sinevalue - 32767) * ((cvfbknob * cvfb)>> 12) >> 12)
                            +
                            ( curphasemod  )
                        )
                    );

    //   All done - stuff the DAC output registers.
    //   WAS::  DAC1RDAT = sine_table [0x00000FFF & ((curbasephase >> 20) + 2048)];
    DAC1RDAT = sine_table [0x00000FFF & (final_phase_fm_feedback + 2048)];
    DAC1LDAT = sine_table [0X00000fff & (final_phase_fm + 2048)];
    return;

}

void delay (int cycles){
    int i;
    for (i = 0; i < cycles; i++);
}
int noop ()
{
    return (1);
}

/*
 *
 */
int main(int argc, char** argv) {

       // Configure Oscillator to operate the device at 40MIPS
    // Fosc= Fin*M/(N1*N2), Fcy=Fosc/2
    // Fosc= 7.37M*43/(2*2)=79.22Mhz for ~40MIPS input clock

    //   Current crystal is 4 MHz so the below actually clocks at 41 MIPS
    //  Yeah overclocking!!!   Set PLLFBD to 79 for an accurate 40 MIPS.
    //   which is 80 MHz on the PLL clock output
      CLKDIVbits.PLLPRE=0;		// N2=2
      PLLFBD = 80 ;			// Set this to 79 for an accurate 80 mips
      CLKDIVbits.PLLPOST=0;		// N1=2
                                        //  4MHz xtal + 0/80/0 yields ~40 MHz inst speed (measured)
//    OSCTUN=0;				// Tune FRC oscillator, if FRC is used

    // Disable Watch Dog Timer
      RCONbits.SWDTEN=0;

    // Wait for PLL to lock
    while(OSCCONbits.LOCK!=1) {};

    //   Set up aux oscillator for the DAC
    ACLKCONbits.SELACLK = 0;    //   Aux oscillator from Main Fosc;
    ACLKCONbits.APSTSCLR = 6;   //  was 6: Divide by 2 - gets 20 MHz to the DAC;
                                //  use 6 to get 83333 hz DAC output rate (measured @ 40 MHz inst)
    ACLKCONbits.ASRCSEL = 0;    //    use primary clock as source (but doesn't matter)

    long int quantum;
    zig = 0;
    iz = 0;
    prevStateRA4 = 1;
                     //  0x1 = ~ 1/15 Hz (1 cycle per 15 seconds)
    quantum = 0x2400;   //  0x24000 = ~ 10 KHz; 0x35000 =~ 15 KHz


    //   Set up which pins are digital in and out
    TRISA = 0x0;    //   Inputs on RA0-1 (ADC 0 and 1)
    TRISAbits.TRISA0 = 1;
    TRISAbits.TRISA1 = 1;

    TRISC = 0x0;    //   Inputs on RC0-1 (ADC 6 and 7)
    TRISCbits.TRISC0 = 1;
    TRISCbits.TRISC1 = 1;

    TRISB = 0x0;    //   Inputs on RB0-3 (ADC 2,3,4,5), RB8-9 (switches)RB12-15 (DACs)
    TRISBbits.TRISB0 = 1;
    TRISBbits.TRISB1 = 1;
    TRISBbits.TRISB2 = 1;
    TRISBbits.TRISB3 = 1;
   // Pins RB6, and RB7 are the LED ports, RB5 is unused
    TRISBbits.TRISB6 = 0;  //  RB6 is heartbeat
    PORTBbits.RB6 = 1;
    TRISBbits.TRISB7 = 0;  //  RB7 is negative frequency
    PORTBbits.RB7 = 1;
    TRISBbits.TRISB8 = 0;  //   Pin RB8 is SYNC OUT,
    TRISBbits.TRISB9 = 1;  //   Pin RB9 is SYNC IN.
    TRISBbits.TRISB12 = 1;
    TRISBbits.TRISB13 = 1;
    TRISBbits.TRISB14 = 1;
    TRISBbits.TRISB15 = 1;

    //   Use port RC6 to see our ADC interrupts, so output...
    TRISCbits.TRISC6 = 0;
    TRISCbits.TRISC7 = 0;

    //   Set up the DACs
    DAC1CONbits.DACEN = 1;  // enable the audio dac
    DAC1CONbits.AMPON = 1;  // enable the output amplifier
    DAC1CONbits.FORM = 0;   //  unsigned data (0 = unsigned data)
    DAC1CONbits.DACFDIV = 3; // divide Fosc to drive interpolator. 3 = 83.333KHz @40MIPS
    DAC1STAT = 0xFFFF;    //  everything on
    DAC1STATbits.LITYPE = 1;  // 0 means interrupt on Left LIFO not full
    DAC1STATbits.RITYPE = 1;  // 0 means interrupt on Right LIFO not full
    DAC1STATbits.LMVOEN = 0;  //  left channel midpoint output off
    DAC1STATbits.RMVOEN = 0;  //  right channel midpoint output off


    //    Set up the ADCs
    //    _ADON = 1;                //  turn on the ADC
    AD1CON1bits.ADON = 1;      // turn on the ADC
    AD1CON1bits.ADSIDL = 0;    //  keep it on even during idle
    AD1CON1bits.ADDMABM = 0;
    AD1CON1bits.AD12B = 1;     //  12-bit mode
    AD1CON1bits.FORM = 0;      //  output as unsigned 12 bit integer
    AD1CON1bits.SSRC = 0;      //  clearing SAMPLE bit ends sampling and starts conversion
    AD1CON1bits.ASAM = 0;      //  sample starts when SAMP = 1; write "0" to start convert
    AD1CON1bits.DONE = 0;      //  set ADC to not be done yet (HW sets to 1 on completion)

    AD1CON2bits.VCFG = 0 ;     // nothing fancy, swing is Vss to Vdd
    AD1CON2bits.CSCNA = 0;     // do not scan inputs
    AD1CON2bits.CHPS = 0;      //  use just 1 ADC channel
    AD1CON2bits.SMPI = 1;      //  increment DMA address on every completion.
    AD1CON2bits.BUFM = 0;      //  start buffer fill at 0
    AD1CON2bits.ALTS = 0;      //  always use sample A

    //   ADC clock must nominally be at least 117 nS long per cycle
    AD1CON3bits.ADCS = 4;      // (was 7)  divide 80 mHz by this plus 1
    AD1CON3bits.ADRC = 0;      // was 1: 1=use the ADC's internal RC clock
    AD1CON3bits.SAMC = 1;      //  Auto sample time bits (ignored here)

    AD1CON4bits.DMABL = 0;     //  DMA buffer length 1 word per analog input


    AD1CHS0bits.CH0NB = 0;     //  channel B negative input is Vref-
    AD1CHS0bits.CH0SB = 0;     //  channel B pos input is input AD0
    AD1CHS0bits.CH0NA = 0;     //  channel A negative input is Vref-
    AD1CHS0bits.CH0SA = 0;     //  channel A positive input is input AD0

    AD1CSSL = 0xFF;            //  AN0-7 enabled for scan (but scan not used)

    AD1PCFGL = 0xFF00;         //  enable ADC on low 8 AD pins.

    IEC0bits.AD1IE = 1;        //  enable the ADC interrupt.
    IPC3bits.AD1IP = 2;        //  set priority for the ADC interrupt (7 is max)

    //   Zeroize the ADC inputs in preparation for those that aren't updated
    {
        int i;
        for (i = 0; i < 12; i++)
            cvdata[i] = 0;
    }

    AD1CON1bits.ADON = 1;      //  turn on the AD converter itself.
    which_adc = ADC_FIRST;  //  start at the first ADC
    AD1CON1bits.DONE = 0;      //  clear DONE bit.
    AD1CON1bits.SAMP = 1;      //  Kick the ADC to start a conversion
    delay(1);
    AD1CON1bits.SAMP = 0;      //  Kick the ADC to start a conversion


    //   ***   DAC INTERRUPT SET UP STUFF
    //IEC4bits.DAC1LIE = 1;      //  enable the left channel DAC FIFO interrupt
    //IPC19bits.DAC1LIP = 1;

    IEC4bits.DAC1RIE = 1;      //  enable the right channel DAC FIFO interrupt
    IPC19bits.DAC1RIP = 5;     //  set the FIFO interrupt priority (7 is max)

    //    The main "WHILE" loop - this does those things that don't have
    //     to be absolutely real time (that is, servicing the ADC and DAC
    //      interrupts.   Things like calculations of various things that are
    //       dependent on the ADC and are needed every DAC tick but don't
    //        necessarily have to change every DAC tick are calculated here.
    //     These are things like the RB6 "hearbeat" LED, RB7 neg Freq.
    //     RB8 and RB9 hard sync in and hard sync out, and the internals:
    //     curpitchval, curpitchincr, and curphasemod.
    //
    while (1)   // Loop Endlessly - Execution is interrupt driven
    {

        //PORTBbits.RB5 = 1;   // DEBUG
        //PORTBbits.RB6 = 0;   // DEBUG
//#define HEARTBEAT_CPU
#ifdef HEARTBEAT_CPU
        iz++;
        PORTBbits.RB6 = 0x1 & (iz >> 17);    // at >>17, and 25 instructions counting loop
                                             // each flash = ~3 MIP of slack
#endif

#define HEARTBEAT_WAVEPHASE
#ifdef HEARTBEAT_WAVEPHASE
        //int bitson[16] = { 0,1,0,1,1,1,0,1,0,1,0,1,0,0,0,1 };
        iz++;
        //   Turns out that if you just toggle a bit fast the LED doesn't
        //   respond AT ALL (capacitance issues; the voltage never gets high
        //   enough to turn on the LED).  So we use TRIS (tristate) instead.
        // PORTBbits.RB6 =  0x3FF > (0x00000FFF & (curbasephase >> 20 ));
        PORTBbits.RB6 = 1;
        TRISBbits.TRISB6 = 0x3FF < (0x00000FFF & (curbasephase >> 20 ));
        // for testing purposes, copy to RB7
        PORTBbits.RB7 = 1;
        TRISBbits.TRISB7 = 0x3FF < (0x00000FFF & (curbasephase >> 20 ));
        //TRISBbits.TRISB6 = bitson [ 0x0F & (curbasephase >> 28)];
#endif

//#define HEARTBEAT_NOTINTERRUPT
#ifdef HEARTBEAT_NOTINTERRUPT
        PORTCbits.RC7 = 1;
        PORTCbits.RC7 = 0;
#endif

//#define NON_LATCHING
#ifdef NON_LATCHING
        char _switch = PORTAbits.RA4; 
        if(prevStateRA4 && !_switch) _PHASEMOD = ~_PHASEMOD & 1u; // toggle phase mod / resolutuion grinding
        prevStateRA4 = _switch;
#endif
        
#define BASELINE_CVPITCHINCR
#ifdef BASELINE_CVPITCHINCR
    curpitchval = cvpitchknob + ( ( cvpitch) - 2048) ;
    //   use exp_table to get exp freq resp.
    curpitchincr = exp_table [
            (curpitchval < 0) ? 0 :
                (curpitchval > 0x0FFF) ? 0x0FFF : curpitchval];

#endif

#define BASELINE_CURFREQMOD
#ifdef BASELINE_CURFREQMOD
    curfreqmod = cvfmknob * (cvfm- 2048) << 5;
    //    Output RB7 high if we have negative frequency
    // PORTBbits.RB7 = 1;
    // TRISBbits.TRISB7 = curpitchincr + curfreqmod < 0 ? 0 : 1;
#endif

#define BASELINE_HARDSYNC
#ifdef BASELINE_HARDSYNC
    //   Do we have a HARD SYNC IN request on pin RB9?
    //   GROT GROT GROT note that this hacky hysteresis could / should
    //   really be done in an interrupt.  Maybe later....
    if (PORTBbits.RB9 == 0 ) // high -> low 
    {
        if (oldhardsync == 1)
        {
            curbasephase = 0; 
            oldhardsync = 0;
        }
    }
    else
        oldhardsync = 1;

    //   And output hard sync out on RB9.
    //PORTBbits.RB9 = ((0x00000FFF &
    //            (final_phase_fm - (27 * (curbasephase >> 20)))) >> 10) == 0;
    PORTBbits.RB8 =  final_phase_fm_feedback > 0x00000800;

#endif

#define BASELINE_CURPHASEMOD
#ifdef BASELINE_CURPHASEMOD
    curphasemod = (((cvpm - 2048) * cvpmknob) >> 10);
                //+ ((curphasemod ) / 2);
    // Output RB7 driven high if we have negative phase
    // PORTBbits.RB7 = 1;
    // TRISBbits.TRISB7 = curpitchincr + (curphasemod << 13) < 0 ? 0 : 1 ;
#endif

    //
        //PORTBbits.RB6 = ! (PORTBbits.RB6);     // <<- 16 instructions counting the loop branch

        // NB: Q106 osc square wave is 5-10 uS, fall time 30 uS
        // Ramp/Sawtooth fast edge time 5 uS (lots of ringing),
        //curbasephase+=16;
        //DAC1DFLT =  DAC1RDAT = DAC1LDAT = 0xFFFF & curbasephase;

//
//        DAC1DFLT = sine_table [0x0000FFF & ( curbasephase >> 20)];
//        DAC1LDAT = sine_table [0x0000FFF & ( curbasephase >> 20)];
//        DAC1RDAT = sine_table [0x0000FFF & ( curbasephase >> 20)];
        //DAC1RDAT = cvpitch << 4;
        //DAC1LDAT = 0xFFFF &(zig >> 4);
        //DAC1LDAT = curbasephase >> 16;

        //     Test: do we get less jitter if we only fill when empty?
        //     Alternative test: check on LFULL (and RFULL)
        //if (! DAC1STATbits.LFULL)
        //    DAC1LDAT = sine_table [0x0000FFF & ( curbasephase >> 20)];
        //if (! DAC1STATbits.LFULL)
        //if (DAC1STATbits.LEMPTY)
        //    DAC1LDAT = curbasephase >> 16;
        //if (zig < 0x2FFF) zag = quantum;
        //if (zig > 0xDFFF) zag = -quantum;
        //zig+=cvpitch << 3;
        //if (zig > 0xFFFFF) zig = zig - 0xFFFFF;
    }
}

